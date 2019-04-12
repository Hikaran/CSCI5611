import java.util.*;

double previousTime;
double currentTime;
double elapsedTime;
double timeFactor = 500.0;

boolean paused = true;
boolean debug = false;
boolean validPath = false;

Random r = new Random();

double obstacleRadius = 2.0;
double agentRadius = 0.5;
double obstacleRadiusCSpace = obstacleRadius + agentRadius;
double agentSpeed = 1.0;

int numPointsPRM = 50;
ArrayList<PVector> points; // PRM points
ArrayList<PVector> obstaclePositions;
ArrayList<Agent> agents;
ArrayList<GraphPoint> graphPoints; // PRM points including start and goal
ArrayList<GraphEdge> graphEdges; // Valid PRM edges
GraphPoint startGraphPoint;
GraphPoint goalGraphPoint;

// Helper variables
PVector ray = new PVector(0,0,0);
PVector rayToCenter = new PVector(0,0,0);
double dist = 0.0;
double delta = 0.0;
double a = 0;
double b = 0;
double c = 0;
double part = 0;
double root = 0;
double secondRoot = 0;
double newCost = 0;
int pathLength = 0;
GraphPoint dummy = new GraphPoint(new PVector(0,0,0));

void setup() {
  size(600,600);
  
  // Initialize agents
  agents = new ArrayList<Agent>();
  Agent firstAgent = new Agent(new PVector(-9,-9,0), new PVector(9,9,0));
  agents.add(firstAgent);
  Agent secondAgent = new Agent(new PVector(9,-9,0), new PVector(-9,9,0));
  agents.add(secondAgent);
  Agent thirdAgent = new Agent(new PVector(-9,9,0), new PVector(9,-9,0));
  agents.add(thirdAgent);
  Agent fourthAgent = new Agent(new PVector(9,9,0), new PVector(-9,-9,0));
  agents.add(fourthAgent);
  
  // Initialize obstacles
  obstaclePositions = new ArrayList<PVector>();
  obstaclePositions.add(new PVector(0,0,0));
  obstaclePositions.add(new PVector(-2,6,0));
  obstaclePositions.add(new PVector(-3,-7,0));
  
  // Initialize list of PRM points
  points = new ArrayList<PVector>(numPointsPRM);
  for (int i = 0; i < numPointsPRM; i++) {
    points.add(new PVector(0,0,0));
  }
  
  // Initialize graph structures
  graphPoints = new ArrayList<GraphPoint>();
  graphEdges = new ArrayList<GraphEdge>();
  populatePRM();
  
  // Determine a path for each agent
  validPath = false;
  for (Agent a : agents) {
    makeGraph(a);
    validPath = findPathUniformCost(a);
    if (validPath == false) {
      a.validPath = false;
    } else {
      a.validPath = true;
    }
  }
  
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
    for (PVector obstaclePosition : obstaclePositions) {
    dist = obstacleRadiusCSpace - p.dist(obstaclePosition);
      if (dist > 0) {
        // Point ray in correct direction
        ray.set(p);
        ray.sub(obstaclePosition);
        
        // Change magnitude of ray
        ray.setMag((float)obstacleRadiusCSpace + 0.1);
        
        // Move point
        p.set(obstaclePosition);
        p.add(ray);
      }
    }
  }
}

class Agent {
  PVector position;
  PVector velocity;
  PVector start;
  PVector goal;
  
  ArrayList<PVector> path;
  boolean validPath;
  boolean finished;
  int targetNode;

  Agent(PVector s, PVector g) {
    this.position = s.copy();
    this.velocity = new PVector(0,0,0);
    this.start = s.copy();
    this.goal = g.copy();
  }
  
  void Draw() {
    ellipse(300+20*this.position.x,300-20*this.position.y,20,20);
  }
  
  void DrawStartAndGoal() {
    point(300+20*this.start.x,300-20*this.start.y);
    point(300+20*this.goal.x,300-20*this.goal.y);
  }
}

class GraphPoint implements Comparable<GraphPoint> {
  PVector position = new PVector(0,0,0);
  double cost;
  ArrayList<GraphPoint> neighbors; // List of neighbors
  GraphPoint previous;
  
  GraphPoint(PVector p) {
    this.position.set(p);
    this.cost = -1;
    this.neighbors = new ArrayList<GraphPoint>();
    this.previous = null;
  }
  
  void AddNeighbor(GraphPoint gp) {
    this.neighbors.add(gp);
  }
  
  @Override
  public int compareTo(GraphPoint other) {
    if (this.cost < other.cost) {
      return -1;
    } else {
      return 1;
    }
  }
}

class GraphEdge {
  PVector first = new PVector(0,0,0);
  PVector second = new PVector(0,0,0);
  
  GraphEdge(PVector a, PVector b) {
    first.set(a);
    second.set(b);
  }
  
  void Draw() {
    line(300+20*first.x,300-20*first.y,300+20*second.x,300-20*second.y);
  }
}

// Check if an edge between two points would go through an obstacle in configuration space
// Assume both points are outside any obstacle all obstacles are circular
boolean testEdge(PVector first, PVector second) {
  for (PVector obstaclePosition : obstaclePositions) {
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
      continue;
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
  }
  
  return false;
}

// Construct graph from PRM points with edges that do not go through obstacles
void makeGraph(Agent agent) {
  // Clear lists
  graphPoints.clear();
  graphEdges.clear();
  
  GraphPoint newPoint;
  GraphEdge newEdge;
  
  // Add start point to graph
  startGraphPoint = new GraphPoint(agent.start);
  startGraphPoint.cost = 0;
  graphPoints.add(startGraphPoint);
  
  // Add goal point to graph
  goalGraphPoint = new GraphPoint(agent.goal);
  graphPoints.add(goalGraphPoint);
  
  // Add randomized points to graph
  for (PVector p : points) {
    newPoint = new GraphPoint(p);
    graphPoints.add(newPoint);
  }
  
  GraphPoint first;
  GraphPoint second;
  // Add all edges to graph that do not go through obstacle in configuration space
  for (int i = 0; i < numPointsPRM+1; i++) {
    first = graphPoints.get(i);
    for (int j = i+1; j < numPointsPRM+2; j++) {
      second = graphPoints.get(j);
      if (!testEdge(first.position,second.position)) {
        // Set graph points as neighbors
        first.AddNeighbor(second);
        second.AddNeighbor(first);
        
        // Add edge to list
        newEdge = new GraphEdge(first.position,second.position);
        graphEdges.add(newEdge);
      }
    }
  }
}

// Find path through graph with Uniform Cost Search
boolean findPathUniformCost(Agent agent) {  
  // Initialize queue
  PriorityQueue<GraphPoint> queue = new PriorityQueue<GraphPoint>();
  
  // Initialize list of explored points
  ArrayList<GraphPoint> exploredPoints = new ArrayList<GraphPoint>();
  
  GraphPoint currentPoint = startGraphPoint;
  while (!currentPoint.equals(goalGraphPoint)) {
    // Add current point to explored list
    exploredPoints.add(currentPoint);
    
    // Update each unexplored neighbor
    for (GraphPoint p : currentPoint.neighbors) {
      // Skip explored neighbors
      if (exploredPoints.contains(p)) {
        continue;
      }
      
      // Calculate cost to reach neighbor
      newCost = currentPoint.cost + currentPoint.position.dist(p.position);
      
      // Update neighbor and queue if new path is shortest path to neighbor
      if (newCost < p.cost || p.cost == -1) {
        // Remove neighbor from queue
        queue.remove(p);
        
        // Update lowest cost
        p.cost = newCost;
        
        // Set current point as new parent
        p.previous = currentPoint;
        
        // Add neighbor back to queue
        queue.offer(p);
      }
    }
    
    // Retrieve next point from queue
    if (queue.peek() != null) {
      currentPoint = queue.poll();
    } else {
      println("Unexpected null!");
      break;
    }
  }
  
  // Make sure path reaches goal
  if (!currentPoint.equals(goalGraphPoint)) {
    println("Could not find path to goal");
    return false;
  }
  
  // Send path to agent
  agent.path = new ArrayList<PVector>();
  while (currentPoint != null) {
    agent.path.add(currentPoint.position);
    currentPoint = currentPoint.previous;
  }
  Collections.reverse(agent.path);
  agent.position.set(agent.path.get(0));
  agent.targetNode = 0;
  return true;
}

void updateSim(double dt) {
  delta = agentSpeed*dt;
  
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
  
  // Draw obstacles
  noStroke();
  fill(255,0,0);
  for (PVector obsPos : obstaclePositions) {
    ellipse(300+20*obsPos.x,300-20*obsPos.y,40*(float)obstacleRadius,40*(float)obstacleRadius);
  }
    
  if (debug) {
    stroke(100);
    strokeWeight(1);
    for (GraphEdge e : graphEdges) {
      e.Draw();
    }
  }
  
  // Draw Paths
  stroke(50,200,100);
  strokeWeight(2);
  for (Agent a : agents) {
    // Skip agents without a valid path
    if (!a.validPath) {
      continue;
    }
    
    pathLength = a.path.size();
    for (int i = 0; i < pathLength-1; i++) {
      line(300+20*a.path.get(i).x,
           300-20*a.path.get(i).y,
           300+20*a.path.get(i+1).x,
           300-20*a.path.get(i+1).y);
    }
  }
  
  // Draw agents
  noStroke();
  fill(25,175,200);
  for (Agent a : agents) {
    a.Draw();
  }
  
  // Draw PRM
  strokeWeight(5);
  stroke(0);
  for (PVector p : points) {
    point(300+20*p.x,300-20*p.y);
  }
  
  // Draw start and goal for each agent
  stroke(0,255,0);
  for (Agent a : agents) {
    a.DrawStartAndGoal();
  }
}

void draw() {
  background(75);
  
  // Update time
  currentTime = millis();
  elapsedTime = currentTime - previousTime;
  previousTime = currentTime;
  
  if (!paused) {
    updateSim(elapsedTime/timeFactor);
    // Benchmarking
    println("Frame Rate: " + frameRate);
  }
  
  drawSim();
}

void keyPressed() {
  switch (key) {
    case ' ':
      // TODO Calculate reset time?
      populatePRM();
      validPath = false;
      for (Agent a : agents) {
        makeGraph(a);
        validPath = findPathUniformCost(a);
        if (validPath == false) {
          a.validPath = false;
        } else {
          a.validPath = true;
        }
      }
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