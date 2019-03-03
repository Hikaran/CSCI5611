import java.util.*;
import queasycam.*;

QueasyCam cam;
PImage img;
PShape pit;

float floor = 300;

float previousTime;
float currentTime;
float elapsedTime;
float timeFactor = 1000.0;

float genRate = 500.0;
ArrayList<Particle> particles = new ArrayList<Particle>();

PVector helper = new PVector(0,0,0);

public class Particle implements Comparable<Particle> {
  PVector position = new PVector(0,0,0);
  PVector velocity = new PVector(0,0,0);
  PVector acceleration = new PVector(0,0,0);
  PVector toCamera = new PVector(0,0,0);
  PVector xAxis = new PVector(0,0,0);
  PVector yAxis = new PVector(0,0,0);
  
  float theta = random(2*PI);
  float r = 50*(1-random(1)*random(1));
  
  float distToCam;
  float lifespan = 10.0;
  
  public Particle() {
    position.set(300 + r*cos(theta), floor - 5, 300 + r * sin(theta));
    velocity.set(0, -10, 0);
  }
  
  public void Update(float dt) {
    velocity.x -= 5*cos(theta)/lifespan*dt;
    velocity.z -= 5*sin(theta)/lifespan*dt;
    PVector delta = velocity.copy();
    delta.mult(dt);
    delta.add(new PVector(0.5-random(1),0.1 - random(0.2),0.5-random(1)));
    position.add(delta);
  }
  
  public void Draw() {
    //stroke(255,255/10*(10-lifespan),0);
    //point(position.x,position.y,position.z);
    
    // Vector pointing from particle to camera
    toCamera = cam.position.copy();
    toCamera.sub(this.position);
    toCamera.normalize();
    
    xAxis = cam.getRight().cross(toCamera);
    xAxis.normalize();
    
    yAxis = xAxis.cross(toCamera);
    yAxis.normalize();
    
    xAxis = xAxis.mult(5);
    yAxis = yAxis.mult(5);
    
    pushMatrix();
    translate(position.x,position.y,position.z);
    beginShape();
    tint(255,200/10*lifespan,100,255/10*lifespan);
    texture(img);
    vertex(0,0,0,0,0);
    vertex(xAxis.x,xAxis.y,xAxis.z,0,img.width);
    vertex(xAxis.x+yAxis.x,xAxis.y+yAxis.y,xAxis.z+yAxis.z,img.height,img.width);
    vertex(yAxis.x,yAxis.y,yAxis.z,img.height,0);
    endShape();
    popMatrix();
  }
  
  @Override
  public int compareTo(Particle other) {
    if (this.distToCam > other.distToCam) {
      return -1;
    } else if (this.distToCam < other.distToCam) {
      return 1;
    } else {
      return 0;
    }
  }
}

void setup() {
  size(1000,600,P3D);
  
  cam = new QueasyCam(this);
  cam.speed = 1;
  
  img = loadImage("data/flames.png");
  pit = loadShape("data/firewood.obj");
  
  strokeWeight(0);
  
  previousTime = millis();
}

void updateSim(float dt) {
  int listSize = particles.size();
  for (int i = listSize-1; i >= 0; i--) {
    particles.get(i).Update(dt);
  }
}

void draw() {
  background(0,0,0);
  
  // Update time
  currentTime = millis();
  elapsedTime = currentTime - previousTime;
  previousTime = currentTime;
  
  // Delete expired particles
  int listSize = particles.size();
  for (int i = listSize-1; i >= 0; i--) {
    Particle temp = particles.get(i);
    temp.lifespan -= elapsedTime/timeFactor;
    
    if (temp.lifespan < 0) {
      particles.remove(i);
    }
  }
  
  // Generate new particles
  float newParticleCount = elapsedTime/timeFactor * genRate;
  for (int i = 0; i < newParticleCount; i++) {
    Particle newParticle = new Particle();
    particles.add(newParticle);
  }
  
  // Benchmarking
  println("Particle Count: " + particles.size());
  println("Frame Rate: " + frameRate);
  
  // Update physics
  updateSim(elapsedTime/timeFactor);
    
  // Draw the ground
  pushMatrix();
  translate(500,floor,0);
  fill(50,50,50);
  box(1000,10,1000);
  popMatrix();
    
  // Draw fire pit
  pushMatrix();
  translate(300,280,280);
  scale(5,5,5);
  shape(pit);
  popMatrix();
  
  // Sort particles
  for (Particle p : particles) {
    helper = cam.position.copy();
    helper.sub(p.position);
    p.distToCam = helper.mag();
  }
  
  Collections.sort(particles);
  
  // Draw particles
  for (Particle p : particles) {
    p.Draw();
  }
}