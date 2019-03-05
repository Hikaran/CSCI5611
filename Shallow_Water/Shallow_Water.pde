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

double gravity = 9.8;

double floor = 500.0;

int dimension;
double[][] test = new double[dimension][dimension];

// Helper variables


void setup() {
  size(1000,600,P3D);

  cam = new QueasyCam(this);
  cam.speed = 0.1;
  
  // Set initial conditions of waves
  
    
  println("Press P toggle pausing.");
  println("Press B to toggle debug view.");
  println("Press F to toggle frame rate reporting.");
  println("Press space to reset simulation.");
  
  // Initialize time
  previousTime = millis();
}

void updateSim(double dt) {
  if (paused) {
    return;
  }

  
  // Boundary conditions
}

void drawSim() {
  if (debug) {
    // Draw each particle
    
  } else {
    noStroke();
  // Draw water with triangles
  }
  
  pushMatrix();
  noStroke();
  noFill();
  translate(0,(float)floor,0);
  stroke(0);
  box(500,500,500);
  popMatrix();
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