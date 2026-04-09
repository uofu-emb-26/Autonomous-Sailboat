# How to connect to SparkFun ProRF module

## Step 1

Use this branch

```bash
git pull

git checkout SAMD21_LoRa
```

## Step 2

Install platformio

```bash
pip install platformio
```

or

```bash
brew install platformio
```



## Step 3

Connect to MCU, turn it on, and press `RST` button twice to turn into boot mode

## Step 4

Run:

```bash
pio run -t upload

pio device monitor
```



Now just wait





Author: Adam Welsh
