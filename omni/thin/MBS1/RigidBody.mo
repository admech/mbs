within MBS;

partial model RigidBody

  replaceable KinematicPort OutPort annotation (extent=[-10, -100; 10, -80],
      Placement(transformation(extent={{-20,-100},{20,-60}})));
  parameter SI.Mass m=1 "Mass of the body";
  parameter SI.MomentOfInertia I[3, 3]=[1, 0, 0; 0, 1, 0; 0, 0, 1]
    "Body's central tensor of inertia";
  SI.Position r[3] "Radius vector of masscenter in global coords";
  SI.Velocity v[3] "Velocity vector of masscenter";
  // State select = trick added to debug omni vert wheel
  SI.Acceleration a[3] "Acceleration vector of masscenter";
//  SI.Acceleration a[3](stateSelect = StateSelect.never) "Acceleration vector of masscenter";
  Real q[4] "Quaternion of body orientation";
  SI.AngularVelocity omega[3] "Vector of angular rate in local coords";
  SI.AngularAcceleration[3] epsilon "Vector of angular acceleration";
//  SI.AngularAcceleration[3] epsilon(stateSelect = StateSelect.never) "Vector of angular acceleration";
  SI.Force F[3] "Sum of all forces applied";
  SI.Torque M[3] "Sum of all torques applied";
  Real T[3, 3] "Matrix of rotation";
  Real Active(start=1) "Flag of active dynamics";
  annotation (Icon, Diagram(
      coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
            100}}),         Polygon(points=[-10, -80; -10, -88; 0, -84; -10,
            -80], style(color=10, fillColor=10)),
      graphics));
protected
  Real a1;
  Real a2;
  Real A1[3];
  Real A2[3];
  Real A3[3];
  Real q3[4];
equation
  a1 = q[1];
  A1 = {q[2],q[3],q[4]};
  a2 = 0;
  A2 = {omega[1],omega[2],omega[3]};
  A3 = a1*A2 + a2*A1 + cross(A1, A2);
  q3 = {a1*a2 - A1*A2,A3[1],A3[2],A3[3]};

  der(Active) = 0;
  der(r) = Active*v;
  der(v) = Active*a;
  m*a = F;
  der(q) = Active*0.5*q3;
//  der(q) = Active*0.5*QMult(q, {0,omega[1],omega[2],omega[3]});
  der(omega) = Active*epsilon;
  T = [q[1]^2 + q[2]^2 - q[3]^2 - q[4]^2, 2*(q[2]*q[3] - q[1]*q[4]), 2*(q[2]*
    q[4] + q[1]*q[3]); 2*(q[1]*q[4] + q[2]*q[3]), q[1]^2 - q[2]^2 + q[3]^2 -
    q[4]^2, 2*(q[3]*q[4] - q[1]*q[2]); 2*(q[2]*q[4] - q[1]*q[3]), 2*(q[1]*q[2]
     + q[3]*q[4]), q[1]^2 - q[2]^2 - q[3]^2 + q[4]^2]/(q*q);
  I*epsilon + cross(omega, I*omega) = transpose(T)*M;
  OutPort.r = r;
  OutPort.v = v;
  OutPort.a = a;
  OutPort.T = T;
  OutPort.omega = T*omega;
  OutPort.epsilon = T*epsilon;
end RigidBody;
