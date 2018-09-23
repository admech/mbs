within MbsLite;

partial model RigidBody

  replaceable KinematicPort OutPort;

  parameter SI.Mass                   m = 1                                "Mass of the body";
  parameter SI.MomentOfInertia[3, 3]  I = [ 1, 0, 0;  0, 1, 0;  0, 0, 1 ]  "Body's central tensor of inertia";

  SI.Position[3]             r         "Radius vector of masscenter in global coords";
  SI.Velocity[3]             v         "Velocity vector of masscenter";
  SI.Acceleration[3]         a         "Acceleration vector of masscenter";
  Real[4]                    q         "Quaternion of body orientation";

  SI.AngularVelocity[3]      omega     "Vector of angular rate in local coords";
  SI.AngularAcceleration[3]  epsilon   "Vector of angular acceleration";
  // State select -- trick added to debug omni vert wheel
  // SI.Acceleration a[3](stateSelect = StateSelect.never) "Acceleration vector of masscenter";
  // SI.AngularAcceleration[3]  epsilon(stateSelect = StateSelect.never) "Vector of angular acceleration";

  SI.Force[3]                F         "Sum of all forces applied";
  SI.Torque[3]               M         "Sum of all torques applied";

  Real[3, 3]                 T         "Matrix of rotation";
  Real                       Active(start=1) "Flag of active dynamics";

protected
  Real a1;
  Real a2;
  Real A1[3];
  Real A2[3];
  Real A3[3];
  Real q3[4];

equation
  a1 = q[1];
  a2 = 0;
  A1 = { q[2], q[3], q[4] };
  A2 = { omega[1], omega[2], omega[3] };
  A3 = a1 * A2 + a2 * A1 + cross(A1, A2);
  q3 = { a1 * a2 - A1 * A2, A3[1], A3[2], A3[3] };

  der(Active) = 0;

  der(r) = Active * v;
  der(v) = Active * a;
  m * a = F;

  der(q) = Active * 0.5 * q3; // der(q) = Active * 0.5 * QMult(q, { 0, omega[1], omega[2], omega[3] });
  der(omega) = Active * epsilon;
  T =
    [ q[1]^2 + q[2]^2 - q[3]^2 - q[4]^2,    2 * (q[2] * q[3] - q[1] * q[4]),      2 * (q[2] * q[4] + q[1] * q[3])
    ; 2 * (q[1] * q[4] + q[2] * q[3]),      q[1]^2 - q[2]^2 + q[3]^2 - q[4]^2,    2 * (q[3] * q[4] - q[1] * q[2])
    ; 2 * (q[2] * q[4] - q[1] * q[3]),      2 * (q[1] * q[2] + q[3] * q[4]),      q[1]^2 - q[2]^2 - q[3]^2 + q[4]^2
    ] / (q * q);
  I * epsilon + cross(omega, I * omega) = transpose(T) * M;

  OutPort.r = r;
  OutPort.v = v;
  OutPort.a = a;
  OutPort.T = T;
  OutPort.omega = T * omega;
  OutPort.epsilon = T * epsilon;

end RigidBody;
