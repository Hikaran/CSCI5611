import java.util.*;
import queasycam.*;

QueasyCam cam;

ArrayList<Particle> particles = new ArrayList<Particle>();
ArrayList<Spring> springs = new ArrayList<Spring>();

ArrayList<Particle> anchors = new ArrayList<Particle>();

boolean paused = false;

double previousTime;
double currentTime;
double elapsedTime;
double timeFactor = 500.0;

double gravity = 9.8;
double springStiffness = 1;
double springDampening = 0.001;
double springRestLength = 7.5;
double springInitialLength = 10.0;

int threadLength = 7;
int threadCount = 3;

double floor = 500.0;

void setup() {
  size(1000,600,P3D); //<>//
  
  cam = new QueasyCam(this);
  cam.speed = 1;
  
  // Set up particles
  for (int i = 0; i < threadLength; i++) {
    Particle newParticle = new Particle();
    newParticle.position.set(0,(float)(i*springInitialLength),0);
    newParticle.initialPosition.set(newParticle.position);
    particles.add(newParticle);
  }
  
  for (int i = 0; i < threadLength; i++) {
    Particle newParticle = new Particle();
    newParticle.position.set((float)(i*springInitialLength)+200,(float)(i*springInitialLength),(float)(i*springInitialLength));
    newParticle.initialPosition.set(newParticle.position);
    particles.add(newParticle);
  }
  
  for (int i = 0; i < threadLength; i++) {
    Particle newParticle = new Particle();
    newParticle.position.set((float)(-i*springInitialLength)-200,0,0);
    newParticle.initialPosition.set(newParticle.position);
    particles.add(newParticle);
  }
  
  // Designate anchors
  for (int i = 0; i < threadCount; i++) {
    anchors.add(particles.get(i*threadLength));
  }
  
  // Set up springs
  for (int i = 0; i < threadCount; i++) {
    for (int j = 0; j < threadLength-1; j++) {
      Spring newSpring = new Spring(springStiffness, springDampening, springRestLength,particles.get(i*threadLength+j),particles.get(i*threadLength+j+1));
      springs.add(newSpring);
    }
  }
  
  // Initialize time
  previousTime = millis();
}

public class Particle {
  PVector position = new PVector(0,0,0);
  PVector velocity = new PVector(0,0,0);
  PVector forces = new PVector(0,0,0);
  PVector initialPosition = new PVector(0,0,0);
  double mass = 1;
  
  public Particle() {
  }
  
  public Particle(PVector position, PVector velocity) {
    this.position.set(position);
    this.initialPosition.set(position);
    this.velocity.set(velocity);
  }
  
  public void Update(double dt) {
    PVector deltaV = new PVector(0,0,0);
    PVector delta = new PVector(0,0,0);
    
    // Assume mass is uniformly 1 for simplicity
    // Update velocity
    PVector.mult(this.forces, (float) dt, deltaV);
    this.velocity.add(deltaV);
    
    // Update position
    PVector.mult(this.velocity, (float) dt, delta);
    this.position.add(delta);
  }
  
  public void Draw() {
    point(position.x,position.y,position.z);
  }
  
  public void Debug() {
    println("Position " + this.position);
    println("Velocity " + this.velocity);
    println("Forces " + this.forces);
  }
}

public class Spring {
  Particle firstEnd;
  Particle secondEnd;
  double k;
  double dampFactor;
  double restLen;
  
  public Spring(double k, double dampFactor, double restLen, Particle first, Particle second) {
    this.k = k;
    this.dampFactor = dampFactor;
    this.restLen = restLen;
    this.firstEnd = first;
    this.secondEnd = second;
  }
  
  public void CalculateForce() {
    // F = -kx
    PVector delta = this.firstEnd.position.copy();
    delta.sub(this.secondEnd.position);    
    double springForce = -k * (delta.mag() - this.restLen);
    delta.normalize();
    PVector unit = delta.copy();
    delta.mult((float)springForce);
    
    this.firstEnd.forces.add(delta);
    this.secondEnd.forces.sub(delta);
    
    // F = -kv
    delta = this.firstEnd.velocity.copy();
    delta.sub(this.secondEnd.velocity);
    // Get velocity in direction of spring
    double dotProduct = PVector.dot(unit,delta);
    PVector.mult(unit,(float)(-k*dotProduct),delta);
    
    this.firstEnd.forces.add(delta);
    this.secondEnd.forces.sub(delta);
  }
  
  public void Draw() {
    PVector firstPos = firstEnd.position;
    PVector secondPos = secondEnd.position;
    line(firstPos.x,firstPos.y,firstPos.z,secondPos.x,secondPos.y,secondPos.z);
  }
}

void updateSim(double dt) {
  //println("New timestep");
  
  if (paused) {
    return;
  }
  // Reset forces
  for (Particle p : particles) {
    p.forces.set(0,0,0);
  }
  
  // Calculate force from each spring
  for (Spring s : springs) {
    s.CalculateForce();
  }
  
  // Calculate gravitational acceleration for each point
  for (Particle p : particles) {
    
    p.forces.y += gravity * p.mass;
  }
  
  // Update particle velocities and positions
  for (Particle p : particles) {
    // Keep anchors fixed
    if (anchors.contains(p)) {
      p.forces.set(0,0,0);
      p.velocity.set(0,0,0);
    }
    p.Update(dt);
    //println("Position " + p.position);
  }
}

void drawSim() {
  // Draw each particle
  strokeWeight(11.0);
  stroke(100,200,150);
  for (Particle p : particles) {
    p.Draw();
  }
  
  // Draw each spring
  strokeWeight(2.5);
  stroke(0,0,0);
  for (Spring s: springs) {
    s.Draw();
  }
  
  pushMatrix();
  fill(100,100,100);
  translate(0,(float)floor,0);
  box(1000,10,1000);
  popMatrix();
}

void draw() {
  background(255,255,255);
  
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
  if (key == ' ') {
    for (Particle p : particles) {
      p.position.set(p.initialPosition);
      p.velocity.set(0,0,0);
    }
  } else if (key == 'p') {
    paused = !paused;
  }
}