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
double obstacleRadiusCSpace = obstacleRadius + agentRadius;

int numPointsPRM = 10;
ArrayList<PVector> points; // PRM points
ArrayList<GraphPoint> graphPoints; // PRM points including start and goal
ArrayList<GraphEdge> graphEdges; // Valid PRM edges

// Helper variables
PVector ray = new PVector(0,0,0);
PVector rayToCenter = new PVector(0,0,0);
double dist = 0.0;
double a = 0;
double b = 0;
double c = 0;
double part = 0;
double root = 0;
double secondRoot = 0;

void setup() {
  size(600,600);
  
  // Initialize list of PRM points
  points = new ArrayList<PVector>(numPointsPRM); // PRM points
  for (int i = 0; i < numPointsPRM; i++) {
    points.add(new PVector(0,0,0));
  }
  
  // Initialize graph structures
  graphPoints = new ArrayList<GraphPoint>();
  graphEdges = new ArrayList<GraphEdge>();
  
  populatePRM();
  makeGraph();
  findPathUniformCost();
  
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
    dist = obstacleRadiusCSpace - p.dist(obstaclePosition);
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

class GraphPoint {
  PVector position;
  double cost = -1;
  // Neighbors
  
}

class GraphEdge {
  PVector first = new PVector(0,0,0);
  PVector second = new PVector(0,0,0);
  double dist = 0;
  
  GraphEdge(PVector a, PVector b) {
    first.set(a);
    second.set(b);
    dist = first.dist(second);
  }
  
  void Draw() {
    line(300+20*first.x,300-20*first.y,300+20*second.x,300-20*second.y);
  }
}

// Check if an edge between two points would go through an obstacle in configuration space
// Assume both points are outside the obstacle and obstacle is circular
boolean testEdge(PVector first, PVector second) {
  // Determine ray pointing from first to second
  ray.set(second);
  ray.sub(first);
  
  // Determine ray pointing from first to center of circle
  rayToCenter.set(first);
  rayToCenter.sub(obstaclePosition);
  
  // Calculate quadratic coefficients
  a = ray.dot(ray);
  b = 2 * rayToCenter.dot(ray);
  c = rayToCenter.dot(rayToCenter)-obstacleRadiusCSpace*obstacleRadiusCSpace;
  
  // No collision if all solutions are imaginary
  part = b*b-4*a*c;
  if (part < 0) {
    return false;
  }
  
  // Solve quadratic equation
  part = sqrt((float)part);
  root = (-b+part)/(2*a);
  secondRoot = (-b-part)/(2*a);
  
  // Collision if either root falls between points
  if (root > 0.0 && root < 1.0) {
    return true;
  } else if (secondRoot > 0.0 && secondRoot < 1.0) {
    return true;
  }
  
  return false;
}

// Construct graph from PRM points with edges that do not go through obstacles
void makeGraph() {
  // Clear lists
  graphPoints.clear();
  graphEdges.clear();
  
  GraphEdge newEdge;
  for (PVector p : points) {
    if (!testEdge(start,p)) {
      // Add edge between start and point
      newEdge = new GraphEdge(start,p);
      graphEdges.add(newEdge);
    }
    
    if (!testEdge(goal,p)) {
      // Add edge between goal and point
      newEdge = new GraphEdge(goal,p);
      graphEdges.add(newEdge);
    }
  }
  
  
}

// Find path through graph with Uniform Cost Search
void findPathUniformCost() {
}

void updateSim(double dt) {
}

void drawSim() {
  // Draw boundaries
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
    
  if (debug) {
    stroke(100);
    strokeWeight(1);
    for (GraphEdge e : graphEdges) {
      e.Draw();
    }
  }
  
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
      // TODO Calculate reset time?
      populatePRM();
      makeGraph();
      findPathUniformCost();
      agentPosition.set(start);
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