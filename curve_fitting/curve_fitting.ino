/*
 * 도전과제: 적외선 거리센서 자동 Curve Fitting (curve_fitting.ino)
 * 과제코드: 20C10
 */

#define PIN_IR A0
#define NUM_POINTS 7

// 측정할 실제 거리 데이터 (mm 단위: 0cm, 5cm, 10cm, ..., 30cm)
const float actual_distances[NUM_POINTS] = {0.0, 50.0, 100.0, 150.0, 200.0, 250.0, 300.0};
float sensor_readings[NUM_POINTS];

int poly_degree = 2; // 기본 2차 다항식 (시리얼 입력으로 설정)

// 함수 선언
unsigned int ir_sensor_filtered(unsigned int n, float position, int verbose);
int compare(const void *a, const void *b);
void solve_polynomial_regression(float x[], float y[], int n_data, int degree, float coeff[]);
void wait_for_enter();

void setup() {
  Serial.begin(1000000);
  while (!Serial);

  Serial.println("\n==================================================");
  Serial.println("  IR Distance Sensor Automatic Curve Fitting System");
  Serial.println("==================================================");

  // 1. 차수 설정 입력 받기
  Serial.println("Enter polynomial degree for curve fitting (1, 2, or 3) and press Enter:");
  while (Serial.available() > 0) Serial.read(); // 버퍼 비우기
  
  while (Serial.available() == 0);
  poly_degree = Serial.parseInt();
  if (poly_degree < 1 || poly_degree > 3) poly_degree = 2; // 예외처리
  
  // 입력 버퍼에 남은 개행문자 비우기
  while (Serial.available() > 0) Serial.read();

  Serial.print("Selected Polynomial Degree: ");
  Serial.println(poly_degree);
  Serial.println("--------------------------------------------------");

  // 2. 각 위치별 센서 값 측정 수집
  for (int i = 0; i < NUM_POINTS; i++) {
    Serial.print("\n[Step "); Serial.print(i + 1); Serial.print("/"); Serial.print(NUM_POINTS); Serial.print("] ");
    Serial.print("Place object at ");
    Serial.print((int)actual_distances[i] / 10);
    Serial.println(" cm and press ENTER.");

    wait_for_enter();

    Serial.print("Measuring...");
    // 30회 샘플링 중간값 필터링 사용
    sensor_readings[i] = (float)ir_sensor_filtered(30, 0.5, 0);
    
    Serial.print(" Done! Sensor Raw Reading: ");
    Serial.println(sensor_readings[i]);
  }

  // 3. Curve Fitting 연산 실행 및 방정식 출력
  Serial.println("\n==================================================");
  Serial.println("   Calculated Equation (Copy & Paste to Arduino)  ");
  Serial.println("==================================================");

  float coeff[4] = {0, }; // coefficients [a0, a1, a2, a3]
  solve_polynomial_regression(sensor_readings, actual_distances, NUM_POINTS, poly_degree, coeff);

  // C++ 함수 형태로 시리얼 출력
  Serial.println("float volt_to_distance(unsigned int a_value)");
  Serial.println("{");
  Serial.println("  float x = (float)a_value;");
  Serial.print("  return ");

  if (poly_degree == 1) {
    Serial.print(coeff[0], 4);
    Serial.print(" + ("); Serial.print(coeff[1], 6); Serial.println(" * x);");
  } else if (poly_degree == 2) {
    Serial.print(coeff[0], 4);
    Serial.print(" + ("); Serial.print(coeff[1], 6); Serial.print(" * x)");
    Serial.print(" + ("); Serial.print(coeff[2], 8); Serial.println(" * x * x);");
  } else if (poly_degree == 3) {
    Serial.print(coeff[0], 4);
    Serial.print(" + ("); Serial.print(coeff[1], 6); Serial.print(" * x)");
    Serial.print(" + ("); Serial.print(coeff[2], 8); Serial.print(" * x * x)");
    Serial.print(" + ("); Serial.print(coeff[3], 10); Serial.println(" * x * x * x);");
  }
  Serial.println("}");
  Serial.println("==================================================\n");
}

void loop() {
  // 측정 완료 후 대기
}

// 엔터키 입력 대기 함수
void wait_for_enter() {
  while (Serial.available() > 0) Serial.read(); // 기존 버퍼 비우기
  while (Serial.available() == 0); // 입력 대기
  delay(100);
  while (Serial.available() > 0) Serial.read(); // 입력 수신 후 개행문자 처리
}

// ----------------------------------------------------
// 최소제곱법 다항식 회귀분석 연산 (Gauss Elimination)
// ----------------------------------------------------
void solve_polynomial_regression(float x[], float y[], int n_data, int degree, float coeff[]) {
  int m = degree + 1;
  float X[2 * m];
  for (int i = 0; i < 2 * m; i++) {
    X[i] = 0;
    for (int j = 0; j < n_data; j++) {
      X[i] += pow(x[j], i);
    }
  }

  float B[m][m + 1];
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < m; j++) {
      B[i][j] = X[i + j];
    }
  }

  float Y[m];
  for (int i = 0; i < m; i++) {
    Y[i] = 0;
    for (int j = 0; j < n_data; j++) {
      Y[i] += pow(x[j], i) * y[j];
    }
    B[i][m] = Y[i];
  }

  // Gauss Elimination
  for (int i = 0; i < m; i++) {
    for (int k = i + 1; k < m; k++) {
      if (abs(B[i][i]) < abs(B[k][i])) {
        for (int j = 0; j <= m; j++) {
          float temp = B[i][j];
          B[i][j] = B[k][j];
          B[k][j] = temp;
        }
      }
    }
  }

  for (int i = 0; i < m - 1; i++) {
    for (int k = i + 1; k < m; k++) {
      float t = B[k][i] / B[i][i];
      for (int j = 0; j <= m; j++) {
        B[k][j] -= t * B[i][j];
      }
    }
  }

  for (int i = m - 1; i >= 0; i--) {
    coeff[i] = B[i][m];
    for (int j = i + 1; j < m; j++) {
      coeff[i] -= B[i][j] * coeff[j];
    }
    coeff[i] /= B[i][i];
  }
}

// ----------------------------------------------------
// IR 센서 필터링 함수
// ----------------------------------------------------
int compare(const void *a, const void *b) {
  return (*(unsigned int *)a - *(unsigned int *)b);
}

unsigned int ir_sensor_filtered(unsigned int n, float position, int verbose) {
  unsigned int *ir_val, ret_val;

  if ((n == 0) || (n > 100) || (position < 0.0) || (position > 1.0)) return 0;
  if (position == 1.0) position = 0.999;

  ir_val = (unsigned int *)malloc(sizeof(unsigned int) * n);
  if (ir_val == NULL) return 0;

  for (int i = 0; i < n; i++) {
    ir_val[i] = analogRead(PIN_IR);
    delay(5);
  }

  qsort(ir_val, n, sizeof(unsigned int), compare);
  ret_val = ir_val[(unsigned int)(n * position)];
  free(ir_val);

  return ret_val;
}
