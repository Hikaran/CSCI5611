import java.util.*;
import queasycam.*;

QueasyCam cam;

ArrayList<Particle> particles = new ArrayList<Particle>();
ArrayList<Spring> springs = new ArrayList<Spring>();

ArrayList<PVector> positions = new ArrayList<PVector>();
ArrayList<Particle> anchors = new ArrayList<Particle>();

boolean paused = true;
boolean debug = false;
boolean showFrameRate = true;

double previousTime;
double currentTime;
double elapsedTime;
double timeFactor = 100000.0;

double gravity = 9.8;
double combinedAirResistanceFactor = 0.001;

double springStiffness = 500;
double springDampening = 10;
double springRestLength = 0.5;
double springInitialLength = 0.5;

int threadLength = 30;
int loopCount = 50;
int numLoops = 0;

double floor = 500.0;

PVector spherePosition = new PVector(10,10,10);
PVector sphereVelocity = new PVector(0,0,0);
PVector sphereInitialPosition = spherePosition.copy();
double sphereRadius = 2.5;
double speedDelta = 1.0;
float maxSpeed = 5.0;

PImage img;

// Helper variables
PVector ray = new PVector(0,0,0);
PVector u = new PVector(0,0,0);
PVector v = new PVector(0,0,0);
PVector normal = new PVector(0,0,0);
PVector unit = new PVector(0,0,0);
double dotProduct = 0.0;
double crossProduct = 0.0;
double result = 0.0;

void setup() {
  size(1000,600,P3D);
  textureMode(NORMAL);
  
  cam = new QueasyCam(this);
  cam.speed = 0.1;
  
  // Set up particles
  for (int i = 0; i < threadLength; i++) {
    for (int j = 0; j < threadLength; j++) {
      Particle newParticle = new Particle();
      newParticle.position.set((float)springInitialLength*i,0,(float)springInitialLength*j);
      newParticle.initialPosition.set(newParticle.position);
      particles.add(newParticle);
      positions.add(newParticle.position);
    }
  }
  
  // Designate anchors
  for (int i = 0; i < threadLength; i++) {
    anchors.add(particles.get(i));
  }
  
  // Set up springs
  for (int i = 0; i < threadLength-1; i++) {
    for (int j = 0; j < threadLength; j++) {
      Spring newSpring = new Spring(springStiffness, springDampening, springRestLength,particles.get(j*threadLength+i),particles.get(j*threadLength+i+1));
      springs.add(newSpring);
    }
  }
  
  for (int i = 0; i < threadLength; i++) {
    for (int j = 0; j < threadLength-1; j++) {
      Spring newSpring = new Spring(springStiffness, springDampening, springRestLength,particles.get(j*threadLength+i),particles.get((j+1)*threadLength+i));
      springs.add(newSpring);
    }
  }
  
  // Second order springs
  for (int i = 0; i < threadLength-2; i++) {
    for (int j = 0; j < threadLength; j++) {
      Spring newSpring = new Spring(springStiffness/2, springDampening, springRestLength*2,particles.get(j*threadLength+i),particles.get(j*threadLength+i+2));
      springs.add(newSpring);
    }
  }
  
  for (int i = 0; i < threadLength; i++) {
    for (int j = 0; j < threadLength-2; j++) {
      Spring newSpring = new Spring(springStiffness/2, springDampening, springRestLength*2,particles.get(j*threadLength+i),particles.get((j+2)*threadLength+i));
      springs.add(newSpring);
    }
  }
  
  // Third order springs
  for (int i = 0; i < threadLength-5; i++) {
    for (int j = 0; j < threadLength; j++) {
      Spring newSpring = new Spring(springStiffness/4, springDampening, springRestLength*5,particles.get(j*threadLength+i),particles.get(j*threadLength+i+5));
      springs.add(newSpring);
    }
  }
  
  for (int i = 0; i < threadLength; i++) {
    for (int j = 0; j < threadLength-5; j++) {
      Spring newSpring = new Spring(springStiffness/4, springDampening, springRestLength*5,particles.get(j*threadLength+i),particles.get((j+5)*threadLength+i));
      springs.add(newSpring);
    }
  }
  
  // Fourth order springs
  for (int i = 0; i < threadLength-10; i++) {
    for (int j = 0; j < threadLength; j++) {
      Spring newSpring = new Spring(springStiffness/8, springDampening, springRestLength*10,particles.get(j*threadLength+i),particles.get(j*threadLength+i+10));
      springs.add(newSpring);
    }
  }
  
  for (int i = 0; i < threadLength; i++) {
    for (int j = 0; j < threadLength-10; j++) {
      Spring newSpring = new Spring(springStiffness/8, springDampening, springRestLength*10,particles.get(j*threadLength+i),particles.get((j+10)*threadLength+i));
      springs.add(newSpring);
    }
  }
  
  img = loadImage("data/wave.jpg");
  
  println("Press P toggle pausing.");
  println("Press B to toggle debug view.");
  println("Press F to toggle frame rate reporting.");
  println("Press space to reset simulation to initial conditions.");
  println("Up/Down arrows change x velocity of sphere.");
  println("Left/Right arrows change z velocity of sphere.");
  println("X/Z keys change y velocity of sphere.");
  println("Control camera with WASD as well as Q and E keys.");
  
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
    
    // Update velocity
    PVector.mult(this.forces, (float) (dt/this.mass), deltaV);
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
    unit.set(delta);
    delta.mult((float)springForce);
    
    this.firstEnd.forces.add(delta);
    this.secondEnd.forces.sub(delta);
    
    // F = -kv
    delta.set(this.firstEnd.velocity);
    delta.sub(this.secondEnd.velocity);
    // Get velocity in direction of spring
    dotProduct = PVector.dot(unit,delta);
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
  
  // Calculate forces from each spring
  for (Spring s : springs) {
    s.CalculateForce();
  }
  
  // Calculate gravitational acceleration for each point
  for (Particle p : particles) {
    p.forces.y += gravity * p.mass;
  }
  
  /**
  // Calculate air resistance
  for (int i = 0; i < threadLength-1; i++) {
    for (int j = 0; j < threadLength-1; j++) {
      int current = i*threadLength+j;

      // Air resistance for first triangle in quad
      // Get average velocity of nodes
      PVector avgVelocity = particles.get(current).velocity;
      avgVelocity.add(particles.get(current+1).velocity);
      avgVelocity.add(particles.get(current+threadLength).velocity);
      avgVelocity.div(3.0);
      
      if (avgVelocity.mag() == 0.0) {
        continue;
      }
      
      // Calculate normal vector of triangle
      u.set(positions.get(current));
      u.sub(positions.get(current+1));
      v.set(positions.get(current));
      v.sub(positions.get(current+threadLength));
      u.cross(v,normal);
            
      // dotProduct = v dot normal
      dotProduct = avgVelocity.dot(normal);
      
      if (dotProduct < 0.0) {
        dotProduct = -dotProduct;
      }
      
      // result = -factor*|v|*(v dot normal)
      result = -combinedAirResistanceFactor*avgVelocity.mag()*dotProduct;
      
      // F = -0.5*p*c_d*|v|^2*a*n
      //normal.normalize();
      PVector.mult(normal, (float) result, ray);
      particles.get(current).forces.add(ray);
      particles.get(current+1).forces.add(ray);
      particles.get(current+threadLength).forces.add(ray);
            
      // Air resistance for second triangle in quad
    }
  }
  */
  
  // Move sphere
  PVector delta = sphereVelocity.copy();
  delta.mult((float)dt);
  spherePosition.add(delta);
  delta = null;
  
  // Handle collisions between cloth and ball
  for (Particle p : particles) {
    PVector.sub(p.position, spherePosition, ray);
    
    if (ray.mag() < (sphereRadius+0.2)) {
      // Move point back outside sphere
      ray.setMag((float)(sphereRadius+0.2));
      PVector.add(spherePosition,ray,p.position);
      
      // Move sphere
      ray.setMag(0.1);
      sphereVelocity.sub(ray);
      if (sphereVelocity.x > maxSpeed) {
        sphereVelocity.x = maxSpeed;
      } else if (sphereVelocity.x < -maxSpeed) {
        sphereVelocity.x = -maxSpeed;
      }
      
      if (sphereVelocity.y > maxSpeed) {
        sphereVelocity.y = maxSpeed;
      } else if (sphereVelocity.y < -maxSpeed) {
        sphereVelocity.y = -maxSpeed;
      }
      
      if (sphereVelocity.z > maxSpeed) {
        sphereVelocity.z = maxSpeed;
      } else if (sphereVelocity.z < -maxSpeed) {
        sphereVelocity.z = -maxSpeed;
      }
      
      // Reflect velocity
      ray.normalize();
      dotProduct = ray.dot(p.velocity);
      ray.setMag((float)(dotProduct*2));
      p.velocity.sub(ray);
    }
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
  if (debug) {
    // Draw each particle
    strokeWeight(11.0);
    stroke(100,200,150);
    for (Particle p : particles) {
      p.Draw();
    }
    
    // Draw each spring
    strokeWeight(2.5);
    stroke(255);
    for (Spring s: springs) {
      s.Draw();
    }
  } else {
    noStroke();

    // Texture each rectangle
    int currentCorner;
    float interval = 1/((float)(threadLength-1));
    
    for (int i = 0; i < threadLength-1; i++) {
      for (int j = 0; j < threadLength-1; j++) {        
        beginShape();
        texture(img);    
        currentCorner = i*threadLength+j;
        vertex(positions.get(currentCorner).x,positions.get(currentCorner).y,positions.get(currentCorner).z,interval*j,interval*i);
        
        currentCorner++;
        vertex(positions.get(currentCorner).x,positions.get(currentCorner).y,positions.get(currentCorner).z,interval*(j+1),interval*i);
        
        currentCorner += threadLength;
        vertex(positions.get(currentCorner).x,positions.get(currentCorner).y,positions.get(currentCorner).z,interval*(j+1),interval*(i+1));
        endShape();

        beginShape();
        texture(img);
        currentCorner = i*threadLength+j;
        vertex(positions.get(currentCorner).x,positions.get(currentCorner).y,positions.get(currentCorner).z,interval*j,interval*i);
        
        currentCorner += threadLength+1;
        vertex(positions.get(currentCorner).x,positions.get(currentCorner).y,positions.get(currentCorner).z,interval*(j+1),interval*(i+1));
        
        currentCorner--;
        vertex(positions.get(currentCorner).x,positions.get(currentCorner).y,positions.get(currentCorner).z,interval*j,interval*(i+1));
        endShape();
      }
    }
  }
  
  pushMatrix();
  fill(100,200,200);
  translate(spherePosition.x,spherePosition.y,spherePosition.z);
  sphere((float)sphereRadius);
  popMatrix();
  
  pushMatrix();
  noStroke();
  fill(100,100,100);
  translate(0,(float)floor,0);
  box(1000,10,1000);
  popMatrix();
}

void draw() {
  background(0);
  
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
      for (Particle p : particles) {
        p.position.set(p.initialPosition);
        p.velocity.set(0,0,0);
      }
      spherePosition.set(sphereInitialPosition);
      sphereVelocity.set(0,0,0);
      break;
    case 'p':
      paused = !paused;
      break;
    case 'b':
      debug = !debug;
      break;
    case 'f':
      showFrameRate = !showFrameRate;
    case 'x':
      sphereVelocity.y += speedDelta;
      if (sphereVelocity.y > maxSpeed) {
        sphereVelocity.y = maxSpeed;
      }
      println("Sphere velocity: " + sphereVelocity);
      break;
    case 'z':
      sphereVelocity.y -= speedDelta;
      if (sphereVelocity.y < -maxSpeed) {
        sphereVelocity.y = -maxSpeed;
      }
      println("Sphere velocity: " + sphereVelocity);
      break;
    case CODED:
      switch(keyCode) {
        case UP:
          sphereVelocity.x += speedDelta;
          if (sphereVelocity.x > maxSpeed) {
            sphereVelocity.x = maxSpeed;
          }
          println("Sphere velocity: " + sphereVelocity);
          break;
        case DOWN:
          sphereVelocity.x -= speedDelta;
          if (sphereVelocity.x < -maxSpeed) {
            sphereVelocity.x = -maxSpeed;
          }
          println("Sphere velocity: " + sphereVelocity);
          break;
        case LEFT:
          sphereVelocity.z += speedDelta;
          if (sphereVelocity.z > maxSpeed) {
            sphereVelocity.z = maxSpeed;
          }
          println("Sphere velocity: " + sphereVelocity);
          break;
        case RIGHT:
          sphereVelocity.z -= speedDelta;
          if (sphereVelocity.z < -maxSpeed) {
            sphereVelocity.z = -maxSpeed;
          }
          println("Sphere velocity: " + sphereVelocity);
          break;
      }
  }
}