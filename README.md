# Sensor

  - 온습도, 먼지 농도, 이산화탄소 농도 감지
  - 센서가 읽은 값의 범위를 나누어 각 명령어를 서버에 전달

# Dust Density(ug/m^3)
  - 센서의 데이터를 3초마다 서버에 전달
  - Case 1 : 0 ~ 30 - Send Command BLUE
  - Case 2 : 31 ~ 80 - Send Command YELLOW
  - Case 3 : 81 ~ 150 - Send Command ORANGE
  - Case 4 : 151 ~ - Send Command RED

# RHT
  - 온도와 습도 데이터를 2초마다 서버에 전달하지만 Dust Density 데이터 전달 우선

# CO2 Density
  - 가스 노출 시 Send Command fire
