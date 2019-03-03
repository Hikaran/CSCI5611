import queasycam.*;

QueasyCam cam;
PShape hydrant;

float floor = 100;
float gravity = 9.8;

float sphereSize = 5.0;
PVector spherePosition = new PVector(150,62,280);
PVector normal = new PVector();

float previousTime;
float currentTime;
float elapsedTime;
float timeFactor = 1000.0;

float genRate = 1000.0;
float particleSize = 10.0;
ArrayList<Particle> particles = new ArrayList<Particle>();

class Particle {
  PVector position = new PVector();
  PVector velocity = new PVector();
  PVector delta = new PVector();
  float red, green, blue;
  float lifespan;
  
  public Particle() {
    float theta = TWO_PI;
    float r = random(5);
    
    position.set(100,floor - 50,300);
    velocity.set(25 + r*cos(theta),r*sin(theta),-random(10));
    lifespan = 10.0;
    red = 50;
    green = 100;
    blue = 200;
  }
  
  public void Update(float dt) {
    velocity.y += gravity * dt;
    delta = velocity.copy();
    delta.mult(dt);
    position.add(delta);

    if (position.y > floor - particleSize) {
      position.y = floor - particleSize;
      velocity.y = velocity.y * -0.5;
      velocity.x = velocity.x * 0.75;
      velocity.z = velocity.z * 0.75;
      
      red += (255-red)/10;
      green += (255-green)/10;
      blue += (255-blue)/10;
    }
  }
  
  public void Draw() {
    stroke(red,green,blue);
    point(position.x,position.y,position.z);
  }
  
  public void Collide() {
    normal = this.position.copy();
    normal.sub(spherePosition);
    float dist = normal.mag();
    normal.normalize();
    
    if (dist < sphereSize+0.1) {
      // Move particle
      position.set(spherePosition);
      position.add(normal.mult(sphereSize+0.1));
      normal.normalize();
      
      // Change velocity
      float dotProduct = normal.dot(this.velocity);
      normal.setMag(-1.75*dotProduct);
      this.velocity.add(normal);
    }
  }
}

void setup() {
  size(1000,600,P3D);
  
  cam = new QueasyCam(this);
  cam.speed = 0.5;
  cam.sensitivity = 0.5;
  
  hydrant = loadShape("data/fire_hydrant.obj");
  
  previousTime = millis();
}

void updateSim(float dt) {
  for (Particle p : particles) {
    p.Update(dt);
  }
}

void draw() {
  background(255,255,255);
  lights();
  
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
  
  // Draw hydrant
  pushMatrix();
  translate(73,floor-20,300);
  pushMatrix();
  translate(19,5,1);
  fill(100);
  box(20);
  popMatrix();
  scale(0.6,0.6,0.6);
  rotateX(PI/2);
  shape(hydrant);
  popMatrix();
  
  // Draw obstacle
  pushMatrix();
  translate(spherePosition.x,spherePosition.y,spherePosition.z);
  fill(200,100,150);
  sphere(sphereSize);
  popMatrix();
  
  // Handle collisions
  for (Particle p : particles) {
    p.Collide();
  }
  
  // Draw particles
  strokeWeight(particleSize);
  for (Particle p : particles) {
    p.Draw();
  }
  strokeWeight(0);
}