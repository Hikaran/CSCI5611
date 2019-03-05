import java.util.*;
import queasycam.*;

QueasyCam cam;

boolean paused = true;
boolean debug = false;
boolean showFrameRate = true;

double previousTime;
double currentTime;
double elapsedTime;

double timeFactor = 1000000.0;
int loopCount = 100;

double g = 9.8;
double dx = 10;

double floor = 500.0;

int dimension = 10;
double[][] h = new double[dimension][dimension];
double[][] hu = new double[dimension][dimension];
double[][] hv = new double[dimension][dimension];

// Cell locations
double[][] x = new double[dimension][dimension];
double[][] z = new double[dimension][dimension];

// Midpoint arrays
double[][] hmx = new double[dimension][dimension-1];
double[][] hum = new double[dimension][dimension-1];
double[][] hmz = new double[dimension-1][dimension];
double[][] hvm = new double[dimension-1][dimension];


void setup() {
  size(1000,600,P3D);

  cam = new QueasyCam(this);
  cam.speed = 1;
  
  // Set initial conditions of waves
  for (int i = 0; i < dimension; i++) {
    for (int j = 0; j < dimension; j++) {
      if (j < 5) {
        h[i][j]=40-10*j+100;
      } else {
        h[i][j]=100;
      }
      x[i][j]=j*dx;
      z[i][j]=i*dx;
    }
  }
    
  println("Press P toggle pausing.");
  println("Press B to toggle debug view.");
  println("Press F to toggle frame rate reporting.");
  
  // Initialize time
  previousTime = millis();
}

void updateSim(double dt) {
  if (paused) {
    return;
  }

  // Update each height and momentum midpoint in x direction
  for (int j = 0; j < dimension-1; j++) {
    hmx[0][j] = 0.5*(h[0][j+1]+h[0][j]) - 0.5*dt/dx*(hu[0][j+1]-hu[0][j]);
    hum[0][j] = 0.5*(hu[0][j+1]+hu[0][j]) - 0.5*dt/dx*(hu[0][j+1]*hu[0][j+1]/h[0][j+1]-hu[0][j]*hu[0][j]/h[0][j]
                                                       +0.5*g*(h[0][j+1]*h[0][j+1]-h[0][j]*h[0][j]));
  }
   //<>//
  // Full step with midpoint flux
  for (int j = 1; j < dimension-1; j++) {
    h[0][j] -= dt/dx*(hum[0][j]-hum[0][j-1]);
  }
  
  for (int j = 1; j < dimension-1; j++) {
    hu[0][j] -= dt/dx*(hum[0][j]*hum[0][j]/hmx[0][j]-hum[0][j-1]*hum[0][j-1]/hmx[0][j-1]
                      +0.5*g*(hmx[0][j]*hmx[0][j]-hmx[0][j-1]*hmx[0][j-1]));
  }
  
  // Boundary conditions
  h[0][0] = h[0][1];
  h[0][dimension-1] = h[0][dimension-2];
  hu[0][0] = -hu[0][1];
  hu[0][dimension-1] = -hu[0][dimension-2];
}

void drawSim() {
  if (debug) {
    // Draw each point
    stroke(100,200,150);
    strokeWeight(10.0);
    for (int i = 0; i < dimension; i++) {
      for (int j = 0; j < dimension; j++) {
        point((float)x[i][j],(float)-h[i][j],(float)z[i][j]);
      }
    }
  } else {
    noStroke();
    // Draw with triangles
  }
  
  /**
  // Draw bounding box
  pushMatrix();
  noStroke();
  noFill();
  translate(0,(float)floor,0);
  stroke(0);
  strokeWeight(1);
  box(500,500,500);
  popMatrix();
  */
}

void draw() {
  background(50);
  
  // Update time
  currentTime = millis();
  elapsedTime = currentTime - previousTime;
  previousTime = currentTime;
  
  for (int i = 0; i < loopCount; i++) {
    updateSim(elapsedTime/timeFactor);
  }
  
  
  drawSim();
  
  if (!paused && showFrameRate) {
    // Benchmarking
    println("Frame Rate: " + frameRate);
  }
}

void keyPressed() {
  switch (key) {
    case ' ':
      break;
    case 'p':
      paused = !paused;
      break;
    case 'b':
      debug = !debug;
      break;
    case 'f':
      showFrameRate = !showFrameRate;
  }
}