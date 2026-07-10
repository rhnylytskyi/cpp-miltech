#define ENABLE_LOG    1
#define ENABLE_DEBUG  0

#if ENABLE_LOG
	#define LOG(msg) std::cout << "[LOG] " << msg << std::endl
#else
	#define LOG(msg)
#endif

#if ENABLE_DEBUG
	#define DEBUG(msg) std::cout << "[DEBUG] " << msg << std::endl
#else
	#define DEBUG(msg)
#endif

#define _USE_MATH_DEFINES

#include <iostream>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <cstring>
#include <iomanip>
#include "json.hpp"

using json = nlohmann::json;
using ordered_json = nlohmann::ordered_json;

enum DroneState { STOPPED, ACCELERATING, DECELERATING, TURNING, MOVING };

struct Coord {
	float x;
	float y;

	// Додавання координат
	Coord operator+(const Coord& other) const {
		Coord result{};
		result.x = x + other.x;
		result.y = y + other.y;
		return result;
	}

	// Віднімання координат
	Coord operator-(const Coord& other) const {
		Coord result{};
		result.x = x - other.x;
		result.y = y - other.y;
		return result;
	}

	// Множення на скаляр
	Coord operator*(float s) const {
		Coord result{};
		result.x = x * s;
		result.y = y * s;
		return result;
	}

	// ділення на скаляр
	Coord operator/ (float s) const {
		Coord result{};
		result.x = x / s;
		result.y = y / s;
		return result;
	}

	// порівняння
	bool operator== (const Coord& other) const {
		return (x == other.x && y == other.y);
	}

	bool operator!= (const Coord& other) const {
		return (x != other.x || y != other.y);
	}

	// довжина вектора (hypot)
	float length(Coord c) const {
		return std::hypot(c.x - x, c.y - y);
	}

	// одиничний вектор
	Coord normalize(Coord c) const {
		float len = length(c);
		if (len > 0) {
			return { (c.x - x) / len, (c.y - y) / len };
		}
		return { 0, 0 }; // Повертаємо нульовий, якщо вектор нульовий
	}
};

struct AmmoParams {
	char name[32];
	float mass; 	// маса (кг)
	float drag; 	// коефіцієнт опору
	float lift; 	// коефіцієнт підйому
};

struct DroneConfig {
	Coord startPos;     	// початкова позиція (x, y)
	float altitude;     	// висота
	float initialDir;   	// початковий напрямок (рад)
	float attackSpeed;  	// швидкість атаки (м/с)
	float accelPath;    	// шлях розгону (м)
	char  ammoName[32]; 	// обрані боєприпаси
	float arrayTimeStep;	// крок часу масиву цілей
	float simTimeStep;  	// крок симуляції
	float hitRadius;    	// радіус влучення
	float angularSpeed; 	// кутова швидкість (рад/с)
	float turnThreshold;	// поріг повороту (рад)
};

struct SimStep {
	Coord pos;          	// позиція дрона
	float direction;    	// напрямок (рад)
	int   state;        	// стан автомата (0-4)
	int   targetIdx;    	// індекс поточної цілі
	Coord dropPoint;    	// точка скиду (куди летить дрон)
	Coord aimPoint;     	// куди впаде бомба (якщо скинути зараз)
	Coord predictedTarget;  // прогнозована позиція цілі
};

float getDistance(const Coord& a, const Coord& b) {
	return a.length(b);
}

AmmoParams* getAmmo() {
	std::ifstream fin("ammo.json");
	if (!fin.is_open()) {
		std::cerr << "Error: Could not open ammo file!" << std::endl;
		return nullptr;
	}
	json j; fin >> j;
	fin.close();

	const int ammoCount = j.size();
	// Масив боєприпасів
	AmmoParams* ammo = new AmmoParams[ammoCount];

	for (int i = 0; i < ammoCount; ++i) {
		strncpy_s(ammo[i].name, j[i]["name"].get<std::string>().c_str(), 31);
		ammo[i].mass = j[i]["mass"];
		ammo[i].drag = j[i]["drag"];
		ammo[i].lift = j[i]["lift"];
	}

	return ammo; // викликач відповідає за delete[]!
}

int findAmmoIndexByName(const AmmoParams ammo[], std::string name) {
	for (int i = 0; ammo[i].name != ""; i++) {
		if (ammo[i].name == name) {
			return i;
		}
	}

	return -1;
}

DroneConfig getDroneConfig() {
	std::ifstream fin("config.json");
	json j; fin >> j;
	fin.close();

	DroneConfig config{
		.startPos = {j["drone"]["position"]["x"], j["drone"]["position"]["y"]},
		.altitude = j["drone"]["altitude"],
		.initialDir = j["drone"]["initialDirection"],
		.attackSpeed = j["drone"]["attackSpeed"],
		.accelPath = j["drone"]["accelerationPath"],
		//.ammoName = {}, // буде заповнено нижче
		.arrayTimeStep = j["targetArrayTimeStep"],
		.simTimeStep = j["simulation"]["timeStep"],
		.hitRadius = j["simulation"]["hitRadius"],
		.angularSpeed = j["drone"]["angularSpeed"],
		.turnThreshold = j["drone"]["turnThreshold"],
	};

	strncpy_s(config.ammoName, j["ammo"].get<std::string>().c_str(), 31);

	return config; // викликач відповідає за delete!
}

// Функція для розрахунку часу падіння (TOF) через формулу Кардано
float calcTimeOfFlight(float zd, float v0, const AmmoParams& ammoParams) {
	const float g = 9.81f;

	float m = ammoParams.mass;
	float d = ammoParams.drag;
	float l = ammoParams.lift;

	float a = d * g * m - 2 * d * d * l * v0;
	float b = -3 * g * m * m + 3 * d * l * m * v0;
	float c = 6 * m * m * zd;
	float p = -b * b / (3 * a * a);
	float q = (2 * b * b * b) / (27 * a * a * a) + c / a;

	if (p >= 0) {
		std::cerr << "Error: Not real solution for time of flight." << std::endl;
		return 1;
	}
	if (std::fabs((3 * q / (2 * p)) * std::sqrt(-3 / p)) > 1) {
		std::cerr << "Error: Not real solution for time of flight." << std::endl;
		return 1;
	}

	float phi = std::acos((3 * q / (2 * p)) * std::sqrt(-3 / p));
	float t = 2 * std::sqrt(-p / 3) * std::cos((phi + 4 * M_PI) / 3) - b / (3 * a);

	return t;
}

// Функція для розрахунку горизонтальної відстані
float calcHDistance(float t, float v0, const AmmoParams& ammoParams) {
	const float g = 9.81f;

	float m = ammoParams.mass;
	float d = ammoParams.drag;
	float l = ammoParams.lift;

	float t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t;
	float l2 = l * l, l3 = l2 * l, l4 = l3 * l;
	float d2 = d * d, d3 = d2 * d, d4 = d3 * d;
	float m2 = m * m, m3 = m2 * m, m4 = m3 * m;

	float h = v0 * t
		- (t2 * d * v0) / (2 * m)
		+ (t3 * (6 * d * g * l * m - 6 * d2 * (l2 - 1) * v0)) / (36 * m2)
		+ (t4 * (-6 * d2 * g * l * (1 + l2 + l4) * m + 3 * d3 * l2 * (1 + l2) * v0 + 6 * d3 * l4 * (1 + l2) * v0)) / (36 * pow(1 + l2, 2) * m3)
		+ (t5 * (3 * d3 * g * l3 * m - 3 * d4 * l2 * (1 + l2) * v0)) / (36 * (1 + l2) * m4);

	return h;
}

struct Drone {
	float attackSpeed;
	float angularSpeed;
	float turnThreshold;
	float accelPath;
	float accelRate;
	float simTimeStep;

	DroneState state = STOPPED;
	Coord pos{};
	float speed = 0.0;
	float direction = 0.0;

	float getAngle(const Coord& dropPoint) const {
		return std::atan2(dropPoint.y - pos.y, dropPoint.x - pos.x);
	}

	float getDeltaAngle(const Coord& dropPoint) const {
		float deltaAngle = getAngle(dropPoint) - direction;
		
		// Нормалізація: щоб кут був у межах від -PI до PI
		// (це важливо для пошуку найкоротшого повороту)
		while (deltaAngle > M_PI)  deltaAngle -= 2 * M_PI;
		while (deltaAngle < -M_PI) deltaAngle += 2 * M_PI;

		return deltaAngle;
	}

	bool isNeedTimeToStop(const Coord& dropPoint) const {
		float deltaAngle = getDeltaAngle(dropPoint);
		// Дрону потрібно час на зупинку, якщо кут > turnThreshold і він рухається (speed > 0.1f для уникнення коливань при дуже низьких швидкостях)
		return std::fabs(deltaAngle) > turnThreshold && speed > 0.1f;
	}

	float getTimeToStop(const Coord& dropPoint) const {
		float timeToStop = 0.0f;
		float deltaAngle = getDeltaAngle(dropPoint);

		if (state == TURNING) {
			// Якщо дрон зараз повертається, то враховуємо час на поворот
			float turnTime = std::fabs(deltaAngle) / angularSpeed;
			timeToStop += turnTime;
		}

		if (state == ACCELERATING) {
			// Якщо дрон зараз розганяється, то враховуємо час на досягнення максимальної швидкості
			float accelTime = (attackSpeed - speed) / accelRate;
			timeToStop += accelTime;
		}

		if (state == MOVING) {
			// Якщо дрон вже рухається, то враховуємо час на гальмування до зупинки, поворот, а потім час на розгін до максимальної швидкості
			float decelTime = speed / accelRate;
			float turnTime = std::fabs(deltaAngle) / angularSpeed;
			float accelTime = attackSpeed / accelRate;
			timeToStop += decelTime + turnTime + accelTime;
		}

		return timeToStop;
	}

	void update(const Coord& dropPoint) {
		float angleToDropPoint = getAngle(dropPoint);
		float deltaAngle = getDeltaAngle(dropPoint);

		// Логіка станів (FSM)
		if (std::fabs(deltaAngle) > turnThreshold) {
			DEBUG("  Angle to drop point: " << angleToDropPoint << " rad");
			DEBUG("  Current direction: " << direction << " rad");
			DEBUG("  Delta angle: " << deltaAngle << " rad");

			// Якщо кут > turnThreshold — дрон гальмує до зупинки, а потім починає повертати.
			if (speed > 0.1f) {
				state = DECELERATING;
				DEBUG("  State changed to DECELERATING");
			} else if (state != TURNING) {
				state = TURNING;
				DEBUG("  State changed to TURNING");
			}
		}
		else if (speed < attackSpeed - 0.1f) { // додано невеликий запас для уникнення коливань між ACCELERATING і MOVING
			// Якщо кут ≤ turnThreshold, але швидкість < attackSpeed — дрон розганяється до максимальної швидкості.
			if (state != ACCELERATING) {
				state = ACCELERATING;
				DEBUG("  State changed to ACCELERATING");
			}
		} else {
			// Якщо кут ≤ turnThreshold і швидкість ≥ attackSpeed — дрон рухається до точки скиду.
			if (state != MOVING) {
				state = MOVING;
				DEBUG("  State changed to MOVING");
			}
		}

		// Виконання дій залежно від стану
		switch (state) {
			case DECELERATING:
				// Гальмування до зупинки
				speed = std::max(0.0f, speed - accelRate * simTimeStep);
				break;
			case TURNING:
				// Поворот до точки скиду
				if (deltaAngle > 0) {
					// Якщо deltaAngle > 0, то потрібно повернути за годинниковою стрілкою (збільшити напрямок)
					direction += std::min(angularSpeed * simTimeStep, deltaAngle);
				} else {
					// Якщо deltaAngle < 0, то потрібно повернути проти годинникової стрілки (зменшити напрямок)
					direction -= std::min(angularSpeed * simTimeStep, std::fabs(deltaAngle));
				}
				break;
			case ACCELERATING:
				// як зміниться швидкість після прискорення, але не більше attackSpeed
				speed = std::min(attackSpeed, speed + accelRate * simTimeStep);
				break;
			case MOVING:
				// Якщо кут ≤ turnThreshold — дрон змінює напрямок без зупинки.
				if (std::fabs(deltaAngle) <= turnThreshold) {
					direction = angleToDropPoint;
				}
				// Рух з максимальною швидкістю
				speed = attackSpeed;
				break;
			case STOPPED:
				// Дрон стоїть на місці, не рухається
				break;
		}
		
		// Рух дрона
		if (state == DECELERATING || state == ACCELERATING || state == MOVING) {
			DEBUG("  Speed: " << speed << " m/s");
			// Дрон рухається вперед, змінюючи позицію залежно від напрямку та швидкості
			Coord dir = { std::cos(direction), std::sin(direction) };
			pos = pos + dir * speed * simTimeStep;
			DEBUG("  Position : (" << pos.x << ", " << pos.y << ")");
		}
	}
};

int main() {
	DEBUG("Starting drone simulation...");
	DEBUG("States: STOPPED=0, ACCELERATING=1, DECELERATING=2, TURNING=3, MOVING=4");

	DroneConfig config = getDroneConfig();

	AmmoParams* ammo = getAmmo();
	int ammoIndex = findAmmoIndexByName(ammo, config.ammoName);
	if (ammoIndex == -1) {
		std::cerr << "Error: Cannot find ammo parameters for \"" << config.ammoName << "\"!" << std::endl;
		return 1;
	}
	AmmoParams ammoParams = ammo[ammoIndex];

	std::ifstream ft("targets.json");
	json jt; ft >> jt;
	int tgtCount = jt["targetCount"];
	int timeSteps = jt["timeSteps"];

	Coord** targets = new Coord*[tgtCount];
	for (int i = 0; i < tgtCount; i++) {
		targets[i] = new Coord[timeSteps];
		for (int j = 0; j < timeSteps; j++) {
			targets[i][j].x = jt["targets"][i]["positions"][j]["x"];
			targets[i][j].y = jt["targets"][i]["positions"][j]["y"];
		}
	}

	// Прискорення визначається зі швидкості атаки та шляху розгону
	// a = attackSpeed² / (2 · accelerationPath)
	const float accelRate = config.attackSpeed * config.attackSpeed / (2 * config.accelPath);
	DEBUG("Calculated acceleration rate: " << accelRate);

	Drone drone{
		.attackSpeed = config.attackSpeed,
		.angularSpeed = config.angularSpeed,
		.turnThreshold = config.turnThreshold,
		.accelPath = config.accelPath,
		.accelRate = accelRate,
		.simTimeStep = config.simTimeStep,
		.state = STOPPED,
		.pos = config.startPos,
		.speed = 0.0f,
		.direction = config.initialDir,
	};

	const float tof = calcTimeOfFlight(config.altitude, config.attackSpeed, ammoParams);
	const float hDist = calcHDistance(tof, config.attackSpeed, ammoParams);
	DEBUG("Calculated time of flight: " << tof << "s");
	DEBUG("Calculated horizontal distance: " << hDist << "m");

	int n = 0;
	float currentTime = 0;
	const int maxSteps = 10000;
	SimStep* s = new SimStep[maxSteps]; // Масив кроків симуляції
	while (n < maxSteps) {
		DEBUG("Step " << n << ":");
		DEBUG("  Current time: " << std::fixed << std::setprecision(2) << currentTime << "s");

		float minTimeToDrop = 1e9;
		int closestTargetIdx = -1;
		Coord predictedTarget{};
		Coord dropPoint{};
		bool hasIntermidiatePoint = false;
		Coord intermidiatePoint{};

		// Пошук найкращої цілі
		for (int i = 0; i < tgtCount; i++) {
			float dt = config.arrayTimeStep;
			int idx = (int)(currentTime / dt) % timeSteps;
			int next = (idx + 1) % timeSteps;
			// Інтерполяція позиції цілі між двома відомими точками
			// час кола 60 * arrayTimeStep, тому беремо залишок від ділення поточного часу на цей період і визначаємо, де ми знаходимося між двома точками
			// Наприклад, якщо arrayTimeStep = 0.1, то кожні 6 секунд ми повертаємося до початкової позиції. Якщо currentTime = 6.05, то t = 0.05, і ми знаходимося на 50% шляху між idx і next.
			// frac - через залишок від ділення поточного часу на arrayTimeStep, 
			// що дасть нам час в межах одного кроку, і потім поділити на arrayTimeStep для нормалізації до 0-1. 
			// Це дозволить уникнути проблем з точністю при великих значеннях currentTime.
			float frac = std::fmod(currentTime, dt) / dt;
			float dx = (float)(targets[i][next].x - targets[i][idx].x);
			float dy = (float)(targets[i][next].y - targets[i][idx].y);
			Coord target{ 
				targets[i][idx].x + dx * frac, 
				targets[i][idx].y + dy * frac, 
			};
			float distanceToTarget = getDistance(drone.pos, target);
			float flightTime = distanceToTarget / config.attackSpeed;
			
			hasIntermidiatePoint = hDist + config.accelPath > distanceToTarget;
			if (hasIntermidiatePoint) {
				// Якщо дрону потрібно розігнатися, але до цілі менше, ніж шлях розгону + горизонтальна відстань, то вибираємо проміжну точку на відстані accelPath + hDist від цілі, і спочатку летимо до неї, а потім вже до цілі. Це дозволить нам вчасно розігнатися і не пролетіти повз ціль.
				Coord dir = target.normalize(drone.pos);
				float requiredDist = hDist + config.accelPath;
				intermidiatePoint = (std::fabs(distanceToTarget) < 1e-6) ?
					Coord{ target.x - requiredDist, target.y } :
					Coord{
						target.x - dir.x * requiredDist,
						target.y - dir.y * requiredDist,
				};

				// skip this target for now, because we will try to reach the intermediate point first, and then re-evaluate the target in the next steps
				continue;
			}

			// Якщо дрон вже рухається, то враховуємо час на гальмування до зупинки, а потім час на розгін до максимальної швидкості
			if (drone.isNeedTimeToStop(target)) {
				flightTime += drone.getTimeToStop(target);
			}

			float targetVx = dx / dt;
			float targetVy = dy / dt;
			Coord targetDir = (targets[i][next]).normalize(targets[i][idx]);
			predictedTarget = { 
				target.x + targetDir.x * targetVx * flightTime, 
				target.y + targetDir.y * targetVy * flightTime,
			};

			// Перерахувати балістику до прогнозованої позиції
			dropPoint = predictedTarget + predictedTarget.normalize(drone.pos) * hDist;
			float distanceToDropPoint = getDistance(drone.pos, dropPoint);
			flightTime = distanceToDropPoint / config.attackSpeed; // орієнтовний час прильоту дрона до точки скиду

			// Вибираємо ціль, для якої час досягнення точки скиду (flightTime) є найменшим
			if (flightTime < minTimeToDrop) {
				minTimeToDrop = flightTime;
				closestTargetIdx = i;
			}
		}

		if (hasIntermidiatePoint) {
			drone.update(intermidiatePoint);		
		} else {
			drone.update(dropPoint);
		}
		
		// Запис даних поточного кроку
		Coord dir = { std::cos(drone.direction), std::sin(drone.direction) };
		s[n] = {
			.pos = drone.pos,
			.direction = drone.direction,
			.state = drone.state,
			.targetIdx = closestTargetIdx,
			.dropPoint = dropPoint, // dropPoint = firePoint (результат lead targeting)
			.aimPoint = drone.pos + dir * hDist, // aimPoint = де впаде бомба з поточної позиції
			.predictedTarget = predictedTarget, // predictedTarget = позиція цілі через flightTime
		};

		// Перевірка на досягнення радіусу скиду
		// Якщо дрон знаходиться в межах радіусу скиду від точки скиду, вважаємо, що він досяг мети і завершуємо симуляцію
		float finalDist = getDistance(drone.pos, dropPoint);
		if (finalDist < config.hitRadius) {
			// Ми можемо також перевірити, чи прогнозована позиція цілі (predictedTarget) знаходиться в межах радіусу влучення від точки скиду (dropPoint), щоб врахувати випадки, коли дрон досягає точки скиду, але ціль вже не там через рух.
			if (getDistance(predictedTarget, dropPoint) < config.hitRadius) {
				DEBUG("  Predicted target position is within hit radius. Target neutralized.");
			} else {
				DEBUG("  Predicted target position is outside hit radius. Target may have evaded.");
			}
			
			n++;
			break;
		}

		currentTime += config.simTimeStep;
		n++;
	}

	LOG("Simulation complete. Steps: " << n);

	ordered_json out;
	out["totalSteps"] = n;
	out["steps"] = ordered_json::array();
	for (int i = 0; i < n; ++i) {
		ordered_json step;
		step["position"] = { {"x", s[i].pos.x}, {"y", s[i].pos.y} };
		step["direction"] = s[i].direction;
		step["state"] = s[i].state;
		step["targetIndex"] = s[i].targetIdx;
		step["dropPoint"] = { {"x", s[i].dropPoint.x}, {"y", s[i].dropPoint.y} };
		step["aimPoint"] = { {"x", s[i].aimPoint.x}, {"y", s[i].aimPoint.y} };
		step["predictedTarget"] = { {"x", s[i].predictedTarget.x}, {"y", s[i].predictedTarget.y} };

		out["steps"].push_back(step);
	}

	std::ofstream fout("simulation.json");
	fout << out.dump(2);  // 2 = відступ для читабельності
	fout.close();
	LOG("Output written to simulation.json");

	// added support txt format for backward compatibility
	std::ofstream foutTxt("simulation.txt");
	foutTxt << n << std::endl;
	for (int i = 0; i < n; i++) foutTxt << s[i].pos.x << " " << s[i].pos.y << " "; foutTxt << std::endl;
	for (int i = 0; i < n; i++) foutTxt << s[i].direction << " "; foutTxt << std::endl;
	for (int i = 0; i < n; i++) foutTxt << s[i].state << " "; foutTxt << std::endl;
	for (int i = 0; i < n; i++) foutTxt << s[i].targetIdx << " "; foutTxt << std::endl;
	LOG("Output written to simulation.txt");

	// Звільнення пам'яті
	delete[] ammo;
	ammo = nullptr;

	// Звільнення (зворотній порядок!)
	for (int i = 0; i < tgtCount; i++)
		delete[] targets[i];
	delete[] targets;
	targets = nullptr;

	delete[] s;
	s = nullptr;

	return 0;
}
