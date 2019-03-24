import java.util.*;

double previousTime;
double currentTime;
double elapsedTime;
double timeFactor = 500.0;

boolean paused = true;
boolean debug = false;

Random r = new Random();

PVector start = new PVector(-9,-9,0);
PVector goal = new PVector(9,9,0);
PVector agentPosition = start.copy();
PVector obstaclePosition = new PVector(0,0,0);
double obstacleRadius = 2.0;
double agentRadius = 0.5;

int numPointsPRM = 10;
ArrayList<PVector> points; // PRM points

// Helper variables
PVector ray = new PVector(0,0,0);
double dist = 0.0;

void setup() {
  size(600,600);
  
  // Initialize list of PRM points
  points = new ArrayList<PVector>(numPointsPRM); // PRM points
  for (int i = 0; i < numPointsPRM; i++) {
    points.add(new PVector(0,0,0));
  }
  
  populatePRM();
  
  // Initialize time
  previousTime = millis();
  
  // TODO calculate setup time?
}

// Randomly select valid points in configuration space
void populatePRM() {
  for(PVector p : points) {
    // Randomly select point within boundaries of configuration space
    p.x = r.nextFloat()*19-9.5;
    p.y = r.nextFloat()*19-9.5;
     
    // Keep points outside obstacles in configuration space
    dist = obstacleRadius + agentRadius - p.dist(obstaclePosition);
    if (dist > 0) {
      // Point ray in correct direction
      ray.set(p);
      ray.sub(obstaclePosition);
      
      // Change magnitude of ray
      ray.setMag((float)dist);
      
      // Move point
      p.add(ray);
    }
  }
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
  
  // Draw obstacle
  noStroke();
  fill(255,0,0);
  ellipse(300,300,80,80);
  
  // Draw agent
  fill(25,175,200);
  ellipse(300+20*agentPosition.x,300-20*agentPosition.y,20,20);
  
  // Draw PRM
  strokeWeight(5);
  stroke(0);
  for (PVector p : points) {
    point(300+20*p.x,300-20*p.y);
  }
  
  // Draw start and goal
  stroke(0,255,0);
  point(300+20*start.x,300-20*start.y);
  point(300+20*goal.x,300-20*goal.y);
  
  // Draw Path TODO
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

void keyPressed() {
  switch (key) {
    case ' ':
      populatePRM();
      break;
    case 'p':
      paused = !paused;
      break;
    case 'b':
      debug = !debug;
      break;
    default:
  }
}