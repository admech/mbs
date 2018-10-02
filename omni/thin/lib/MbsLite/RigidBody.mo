within MbsLite;

partial model RigidBody

  replaceable KinematicPort OutPort;

  parameter String                    name = "NOT INITIALIZED";

  parameter SI.Mass                   m (start = inf)      "Mass of the body";
  parameter SI.MomentOfInertia[3, 3]  I (each start = inf) "Central tensor of inertia of the body";

  SI.Position[3]             r (each start = inf) "Radius vector of masscenter in global coords";
  SI.Velocity[3]             v (each start = inf) "Velocity vector of masscenter";
  SI.Acceleration[3]         a                    "Acceleration vector of masscenter";
  Real[4]                    q (each start = inf) "Quaternion of body orientation.  QToT(q) * local = global.  E.g. QRot(pi/6, { 0, 0, 1 }) means this body is rotated clockwise along the third axis (normal to the screen) by pi/6";

  SI.AngularVelocity[3]      omega   (each start = inf) "Vector of angular rate in local coords";
  SI.AngularAcceleration[3]  epsilon                    "Vector of angular acceleration";
  // State select -- trick added to debug omni vert wheel
  // SI.Acceleration a[3](stateSelect = StateSelect.never) "Acceleration vector of masscenter";
  // SI.AngularAcceleration[3]  epsilon(stateSelect = StateSelect.never) "Vector of angular acceleration";

  // should we really specify this explicitly ?
  // SI.Force[3]                F (each start = inf) "Sum of all forces applied";
  // SI.Torque[3]               M (each start = inf) "Sum of all torques applied";
  SI.Force[3]                F "Sum of all forces applied";
  SI.Torque[3]               M "Sum of all torques applied";

  Real[3, 3]                 T "Matrix of rotation. T * local = global, cols = coords of local base in global";
  Real                       Active (start=1) "Flag of active dynamics";

initial algorithm
  assert(CompareReal(q * q, 1), "Quaternion of body orientation should have norm 1, was: " + String(q * q) + ", q = " + StringA(q) + ". Are the initial conditions specified?");
  AssertInitializedS(name,  name, "name");
  AssertInitialized(name,  { m }, "m");
  AssertInitialized(name,  I[1, :], "I[1, :]");
  AssertInitialized(name,  I[2, :], "I[2, :]");
  AssertInitialized(name,  I[3, :], "I[3, :]");
  AssertInitialized(name,  r, "r");
  AssertInitialized(name,  v, "v");
  AssertInitialized(name,  q, "q");
  AssertInitialized(name,  omega, "omega");
  // AssertInitialized(name,  F, "F");
  // AssertInitialized(name,  M, "M");

equation

  der(Active) = 0;

  der(r) = Active * v;
  der(v) = Active * a;
  m * a = F;

  der(q) = Active * 0.5 * QMult(q, { 0, omega[1], omega[2], omega[3] });
  der(omega) = Active * epsilon;
  T = QToT(q);
  I * epsilon + cross(omega, I * omega) = transpose(T) * M;

  OutPort.r = r;
  OutPort.v = v;
  OutPort.a = a;
  OutPort.T = T;
  OutPort.omega = T * omega;
  OutPort.epsilon = T * epsilon;

end RigidBody;
