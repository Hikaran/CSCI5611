import java.util.*;

double previousTime;
double currentTime;
double elapsedTime;
double timeFactor = 500.0;

PVector start = new PVector(-9,-9,0);
PVector goal = new PVector(9,9,0);
PVector currentPosition = start.copy();

void setup() {
  size(600,600);
  
  populatePRM();
  
  // Initialize time
  previousTime = millis();
  
  // TODO calculate setup time?
}

// Randomly select valid points in configuration space
void populatePRM() {
}

void updateSim(double dt) {
}

void drawSim() {
  // Draw bounding box
  fill(255);
  strokeWeight(2.0);
  stroke(0);
  beginShape();
  vertex(100,100);
  vertex(100,500);
  vertex(500,500);
  vertex(500,100);
  endShape(CLOSE);
  
  // Draw goal
  stroke(0,255,0);
  strokeWeight(5);
  point(300+20*goal.x,300-20*goal.y);
  
  // Draw PRM TODO
  
  // Draw Path TODO
  
  // Draw obstacle
  noStroke();
  fill(255,0,0);
  ellipse(300,300,80,80);
  
  // Draw agent
  fill(25,175,200);
  ellipse(300+20*currentPosition.x,300-20*currentPosition.y,20,20);
}

void draw() {
  background(75);
  
  // Update time
  currentTime = millis();
  elapsedTime = currentTime - previousTime;
  previousTime = currentTime;
  
  updateSim(elapsedTime/timeFactor);
  
  drawSim();
  
  // Benchmarking
  // println("Frame Rate: " + frameRate);
}