float radius = 25;
float x = 100;
float y = 100;
float z = 0;
float xVelocity = 200;
float yVelocity = 0;
float gravity = 98;
float floor = 600;
float left = 0;
float right = 1000;

float previousTime;
float currentTime;
float elapsedTime;

void setup() {
  size(1000,600,P3D);
  fill(100,200,150);
  ortho();
  previousTime = millis();
}

void updateSim(float dt) {
  yVelocity += gravity * dt;
  y += yVelocity * dt;
  x += xVelocity * dt;
  
  if (y + radius > floor) {
    yVelocity = -0.9 * yVelocity;
    y = floor - radius;
  }
  
  if (x + radius > right) {
    xVelocity = -0.8*xVelocity;
    x = right - radius;
  }
  
  if (x - radius < left) {
    xVelocity = -0.8*xVelocity;
    x = left + radius;
  }
}

void setupLights() {
  pointLight(100, 200, 255, 0, height, 0);
  ambientLight(100,100,100);
}

void draw() {
  background(255,255,255);
  
  currentTime = millis();
  elapsedTime = currentTime - previousTime;
  previousTime = currentTime;
  updateSim(elapsedTime/500.0);
  
  noStroke();
  setupLights();
  
  pushMatrix();
  translate(x,y,z);
  sphere(radius);
  popMatrix();
}

void keyPressed() {
  if (key == ' ') {
    x = 100;
    y = 100;
    z = 0;
    xVelocity = 200;
    yVelocity = 0;
  }
}