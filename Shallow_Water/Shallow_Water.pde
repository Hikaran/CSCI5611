import java.util.*;
import queasycam.*;

QueasyCam cam;

boolean paused = true;
boolean debug = false;
boolean showFrameRate = true;

double previousTime;
double currentTime;
double elapsedTime;

double timeFactor = 100000.0;
int loopCount = 50;

double g = 9.8;
double dx = 1;
double damp = 0.1;

int dimension = 102;
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
      if (j > 0 && j < 5) {
        h[i][j]=140-10*j;
      } else if (j == 0) {
        h[i][j]=130;
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

  // Update midpoint heights and momentums in x direction
  for (int i = 0; i < dimension; i++) {
    for (int j = 0; j < dimension-1; j++) {
      hmx[i][j] = 0.5*(h[i][j+1]+h[i][j]) - 0.5*dt/dx*(hu[i][j+1]-hu[i][j]);
      hum[i][j] = 0.5*(hu[i][j+1]+hu[i][j]) - 0.5*dt/dx*(hu[i][j+1]*hu[i][j+1]/h[i][j+1]-hu[i][j]*hu[i][j]/h[i][j]
                                                         +0.5*g*(h[i][j+1]*h[i][j+1]-h[i][j]*h[i][j]));
    }
  }
  
  // Update midpoint heights and momentums in z direction
  for (int i = 0; i < dimension-1; i++) {
    for (int j = 0; j < dimension; j++) {
      hmz[i][j] = 0.5*(h[i+1][j]+h[i][j]) - 0.5*dt/dx*(hv[i+1][j]-hv[i][j]);
      hvm[i][j] = 0.5*(hv[i+1][j]+hv[i][j]) - 0.5*dt/dx*(hv[i+1][j]*hv[i+1][j]/h[i+1][j]-hv[i][j]*hv[i][j]/h[i][j]
                                                         +0.5*g*(h[i+1][j]*h[i+1][j]-h[i][j]*h[i][j]));
    }
  }
   //<>//
  // Full step with midpoint flux in x direction
  for (int i = 0; i < dimension; i++) {
    for (int j = 1; j < dimension-1; j++) {
      h[i][j] -= dt/dx*(hum[i][j]-hum[i][j-1]);
      hu[i][j] -= dt/dx*(damp*hu[i][j]+hum[i][j]*hum[i][j]/hmx[i][j]-hum[i][j-1]*hum[i][j-1]/hmx[i][j-1]
                        +0.5*g*(hmx[i][j]*hmx[i][j]-hmx[i][j-1]*hmx[i][j-1]));
    }
  }
  
  // Full step with midpoint flux in z direction
  for (int i = 1; i < dimension-1; i++) {
    for (int j = 0; j < dimension; j++) {
      h[i][j] -= dt/dx*(hvm[i][j]-hvm[i-1][j]);
      hv[i][j] -= dt/dx*(damp+hv[i][j]+hvm[i][j]*hvm[i][j]/hmz[i][j]-hvm[i-1][j]*hvm[i-1][j]/hmz[i-1][j]
                        +0.5*g*(hmz[i][j]*hmz[i][j]-hmz[i-1][j]*hmz[i-1][j]));
    }
  }

  
  // Boundary conditions
  for (int i = 1; i < dimension-1; i++) {
    h[i][0] = h[i][1];
    h[i][dimension-1] = h[i][dimension-2];
    hu[i][0] = -hu[i][1];
    hu[i][dimension-1] = -hu[i][dimension-2];
  }
  for (int j = 1; j < dimension-1; j++) {
    h[0][j] = h[1][j];
    h[dimension-1][j] = h[dimension-2][j];
    hv[0][j] = -hv[1][j];
    hv[dimension-1][j] = -hv[dimension-2][j];
  }
  // Boundary conditions for corner heights
  h[0][0] = 0.5*(h[0][1]+h[1][0]);
  h[0][dimension-1] = 0.5*(h[0][dimension-2]+h[1][dimension-1]);
  h[dimension-1][0] = 0.5*(h[dimension-2][0]+h[dimension-1][1]);
  h[dimension-1][dimension-1] = 0.5*(h[dimension-2][dimension-1]+h[dimension-1][dimension-2]);
  
  // Boundary conditions for corner momentums
  hu[0][0] = hu[0][1];
  hv[0][0] = hv[1][0];
  hu[dimension-1][0] = hu[dimension-1][1];
  hv[dimension-1][0] = hv[dimension-2][0];
  hu[0][dimension-1] = hu[0][dimension-2];
  hv[0][dimension-1] = hv[1][dimension-1];
  hu[dimension-1][dimension-1] = hu[dimension-1][dimension-2];
  hv[dimension-1][dimension-1] = hv[dimension-2][dimension-1];
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
    for (int i = 0; i < dimension-1; i++) {
      for (int j = 0; j < dimension-1; j++) {
        beginShape(TRIANGLE_STRIP);
        fill(50,100,(i*j)%255);
        vertex((float)x[i][j],(float)-h[i][j],(float)z[i][j]);
        vertex((float)x[i][j+1],(float)-h[i][j+1],(float)z[i][j+1]);
        vertex((float)x[i+1][j],(float)-h[i+1][j],(float)z[i+1][j]);
        vertex((float)x[i+1][j+1],(float)-h[i+1][j+1],(float)z[i+1][j+1]);
        endShape();
      }
    }
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