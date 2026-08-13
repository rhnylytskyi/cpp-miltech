#!/bin/bash

CHIP_NAME="${1:-dz22}"
GPIO_SIM_PATH="/sys/kernel/config/gpio-sim"
CHIP_PATH="$GPIO_SIM_PATH/$CHIP_NAME"

echo "==> Видалення gpio-sim чіпа: $CHIP_NAME"

# Перевірка що чіп існує
if [ ! -d "$CHIP_PATH" ]; then
  echo "Помилка: $CHIP_PATH не існує."
  exit 1
fi

# 1. Деактивувати
echo "==> Вимикаємо live..."
echo 0 | sudo tee "$CHIP_PATH/live" > /dev/null

# 2. Видалити всі bank* директорії
for bank in "$CHIP_PATH"/bank*; do
  if [ -d "$bank" ]; then
    echo "==> Видаляємо $bank..."
    sudo rmdir "$bank" || { echo "Помилка: не вдалося видалити $bank"; exit 1; }
  fi
done

# 3. Видалити сам чіп
echo "==> Видаляємо $CHIP_PATH..."
sudo rmdir "$CHIP_PATH" || { echo "Помилка: не вдалося видалити $CHIP_PATH"; exit 1; }

echo "==> Готово!"
echo ""

# Перевірка
echo "==> Залишилось в gpio-sim:"
ls "$GPIO_SIM_PATH/" 2>/dev/null || echo "(порожньо)"

echo ""
echo "==> Пристрої /dev/gpiochip*:"
ls /dev/gpiochip* 2>/dev/null || echo "(немає)"