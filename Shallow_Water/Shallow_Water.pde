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
double[][] h = new double[dimension][dimension]; // cell heights
double[][] hu = new double[dimension][dimension]; // momentum in x direction
double[][] hv = new double[dimension][dimension]; // momentum in z direction

// Cell locations
double[][] x = new double[dimension][dimension];
double[][] z = new double[dimension][dimension];

// Midpoint arrays
double[][] hmx = new double[dimension][dimension-1]; // height midpoints in x direction
double[][] humx = new double[dimension][dimension-1]; // x-momentum midpoints in x direction
double[][] humz = new double[dimension-1][dimension]; // x-momentum midpoints in z direction
double[][] hmz = new double[dimension-1][dimension]; // height midpoints in z direction
double[][] hvmx = new double[dimension][dimension-1]; // z-momentum midpoints in x direction
double[][] hvmz = new double[dimension-1][dimension]; // z-momentum midpoints in z direction

double[][] blue = new double[dimension][dimension]; // blue color component of each cell

void setup() {
  size(1000,600,P3D);

  cam = new QueasyCam(this);
  cam.speed = 1;
  
  // Set initial conditions of waves
  for (int i = 0; i < dimension; i++) {
    for (int j = 0; j < dimension; j++) {
      if (j > 0 && j < 5) {
        h[i][j]=120-5*j;
      } else if (j == 0) {
        h[i][j]=115;
      } else {
        h[i][j]=100;
      }
      x[i][j]=j*dx;
      z[i][j]=i*dx;
    }
  }
  
  for (int i = 0; i < 10; i++) {
    for (int j = 0; j < 10; j++) {
      h[dimension-10+i][dimension-10+j] += 5*(i+j);
    }
  }
  
  // Set cell colors
  for (int i = 0; i < dimension; i++) {
    for (int j = 0; j < dimension; j++) {
      blue[i][j] = 155+(i*j)%100;
    }
  }
  
  println("Press space to reset simulation to initial conditions.");
  println("Press P toggle pausing.");
  println("Press B to toggle debug view.");
  println("Press F to toggle frame rate reporting.");
  println("Control camera with WASD as well as Q and E keys.");
  
  // Initialize time
  previousTime = millis();
}

void resetSim() {
  // Set initial conditions of waves
  for (int i = 0; i < dimension; i++) {
    for (int j = 0; j < dimension; j++) {
      if (j > 0 && j < 5) {
        h[i][j]=120-5*j;
      } else if (j == 0) {
        h[i][j]=115;
      } else {
        h[i][j]=100;
      }
      x[i][j]=j*dx;
      z[i][j]=i*dx;
    }
  }
  
  for (int i = 0; i < 10; i++) {
    for (int j = 0; j < 10; j++) {
      h[dimension-10+i][dimension-10+j] += 5*(i+j);
    }
  }
  
  // Zero out momentums
  for (int i = 0; i < dimension; i++) {
    for (int j = 0; j < dimension; j++) {
      hu[i][j] = 0;
      hv[i][j] = 0;
    }
  }
}

void updateSim(double dt) {
  if (paused) {
    return;
  }

  // Update midpoint heights and momentums in x direction
  for (int i = 0; i < dimension; i++) {
    for (int j = 0; j < dimension-1; j++) {
      hmx[i][j] = 0.5*(h[i][j+1]+h[i][j]) - 0.5*dt/dx*(hu[i][j+1]-hu[i][j]);
      humx[i][j] = 0.5*(hu[i][j+1]+hu[i][j]) - 0.5*dt/dx*(hu[i][j+1]*hu[i][j+1]/h[i][j+1]-hu[i][j]*hu[i][j]/h[i][j]
                                                         +0.5*g*(h[i][j+1]*h[i][j+1]-h[i][j]*h[i][j]));
      hvmx[i][j] = 0.5*(hv[i][j+1]+hv[i][j]) - 0.5*dt/dx*(hu[i][j+1]*hv[i][j+1]/h[i][j+1]-hu[i][j]*hv[i][j]/h[i][j]);
    }
  }
  
  // Update midpoint heights and momentums in z direction
  for (int i = 0; i < dimension-1; i++) {
    for (int j = 0; j < dimension; j++) {
      hmz[i][j] = 0.5*(h[i+1][j]+h[i][j]) - 0.5*dt/dx*(hv[i+1][j]-hv[i][j]);
      hvmz[i][j] = 0.5*(hv[i+1][j]+hv[i][j]) - 0.5*dt/dx*(hv[i+1][j]*hv[i+1][j]/h[i+1][j]-hv[i][j]*hv[i][j]/h[i][j]
                                                         +0.5*g*(h[i+1][j]*h[i+1][j]-h[i][j]*h[i][j]));
      humz[i][j] = 0.5*(hu[i+1][j]+hu[i][j]) - 0.5*dt/dx*(hu[i+1][j]*hv[i+1][j]/h[i+1][j]-hu[i][j]*hv[i][j]/h[i][j]);
    }
  }
   //<>//
  // Full step with x midpoint
  for (int i = 0; i < dimension; i++) {
    for (int j = 1; j < dimension-1; j++) {
      h[i][j] -= dt/dx*(humx[i][j]-humx[i][j-1]);
      hu[i][j] -= dt/dx*(humx[i][j]*humx[i][j]/hmx[i][j]-humx[i][j-1]*humx[i][j-1]/hmx[i][j-1]
                        +0.5*g*(hmx[i][j]*hmx[i][j]-hmx[i][j-1]*hmx[i][j-1])
                        +damp*hu[i][j]);
      hv[i][j] -= dt/dx*(humx[i][j]*hvmx[i][j]/hmx[i][j]-humx[i][j-1]*hvmx[i][j-1]/hmx[i][j-1]);
    }
  }
  
  // Full step with z midpoints
  for (int i = 1; i < dimension-1; i++) {
    for (int j = 0; j < dimension; j++) {
      h[i][j] -= dt/dx*(hvmz[i][j]-hvmz[i-1][j]);
      hv[i][j] -= dt/dx*(hvmz[i][j]*hvmz[i][j]/hmz[i][j]-hvmz[i-1][j]*hvmz[i-1][j]/hmz[i-1][j]
                        +0.5*g*(hmz[i][j]*hmz[i][j]-hmz[i-1][j]*hmz[i-1][j])
                        +damp*hv[i][j]);
      hu[i][j] -= dt/dx*(humz[i][j]*hvmz[i][j]/hmz[i][j]-humz[i-1][j]*hvmz[i-1][j]/hmz[i-1][j]
);
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
    // Draw surface with triangles
    for (int i = 0; i < dimension-1; i++) {
      for (int j = 0; j < dimension-1; j++) {
        beginShape(TRIANGLE_STRIP);
        fill(50,100,(float)blue[i][j]);
        vertex((float)x[i][j],(float)-h[i][j],(float)z[i][j]);
        vertex((float)x[i][j+1],(float)-h[i][j+1],(float)z[i][j+1]);
        fill(50,100,(float)blue[i+1][j+1]);
        vertex((float)x[i+1][j],(float)-h[i+1][j],(float)z[i+1][j]);
        vertex((float)x[i+1][j+1],(float)-h[i+1][j+1],(float)z[i+1][j+1]);
        endShape();
      }
    }
    
    // Draw sides to give illusion of depth
    for (int i = 0; i < dimension-1; i++) {
      beginShape(TRIANGLE_STRIP);
      fill(50,100,150);
      vertex((float)x[i][0],(float)-h[i][0],(float)z[i][0]);
      vertex((float)x[i+1][0],(float)-h[i+1][0],(float)z[i+1][0]);
      fill(0);
      vertex((float)x[i][0],0,(float)z[i][0]);
      vertex((float)x[i+1][0],0,(float)z[i+1][0]);
      endShape();
      
      beginShape(TRIANGLE_STRIP);
      fill(50,100,150);
      vertex((float)x[i][dimension-1],(float)-h[i][dimension-1],(float)z[i][dimension-1]);
      vertex((float)x[i+1][dimension-1],(float)-h[i+1][dimension-1],(float)z[i+1][dimension-1]);
      fill(0);
      vertex((float)x[i][dimension-1],0,(float)z[i][dimension-1]);
      vertex((float)x[i+1][dimension-1],0,(float)z[i+1][dimension-1]);
      endShape();
      
      beginShape(TRIANGLE_STRIP);
      vertex((float)x[i][0],0,(float)z[i][0]);
      vertex((float)x[i+1][0],0,(float)z[i+1][0]);
      vertex((float)x[i][dimension-1],0,(float)z[i][dimension-1]);
      vertex((float)x[i+1][dimension-1],0,(float)z[i+1][dimension-1]);
      endShape();
    }
    for (int j = 0; j < dimension-1; j++) {
      beginShape(TRIANGLE_STRIP);
      fill(50,100,150);
      vertex((float)x[0][j],(float)-h[0][j],(float)z[0][j]);
      vertex((float)x[0][j+1],(float)-h[0][j+1],(float)z[0][j+1]);
      fill(0);
      vertex((float)x[0][j],0,(float)z[0][j]);
      vertex((float)x[0][j+1],0,(float)z[0][j+1]);
      endShape();
      
      beginShape(TRIANGLE_STRIP);
      fill(50,100,150);
      vertex((float)x[dimension-1][j],(float)-h[dimension-1][j],(float)z[dimension-1][j]);
      vertex((float)x[dimension-1][j+1],(float)-h[dimension-1][j+1],(float)z[dimension-1][j+1]);
      fill(0);
      vertex((float)x[dimension-1][j],0,(float)z[dimension-1][j]);
      vertex((float)x[dimension-1][j+1],0,(float)z[dimension-1][j+1]);
      endShape();
    }
  }
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
      resetSim();
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