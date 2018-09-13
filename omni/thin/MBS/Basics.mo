within MBS;
package Basics

  import SI = Modelica.SIunits;

  connector KinematicPort
    // Radius vector of masscenter
    SI.Position r[3];
    // Velocity vector of masscenter
    SI.Velocity v[3];
    // Acceleration vector of masscenter
    SI.Acceleration a[3];
    // Matrix of rotation
    Real T[3, 3];
    // Vector of angular rate
    SI.AngularVelocity omega[3];
    // Vector of angular acceleration
    SI.AngularAcceleration epsilon[3];
  end KinematicPort;
  
  connector WrenchPort
    SI.Position P[3];
    SI.Force F[3];
    SI.Torque M[3];
  end WrenchPort;
  
  function QMult
    input Real q1[4];
    input Real q2[4];
    output Real q3[4];
  protected
    Real a1;
    Real a2;
    Real A1[3];
    Real A2[3];
    Real A3[3];
  algorithm
    a1 := q1[1];
    A1 := {q1[2],q1[3],q1[4]};
    a2 := q2[1];
    A2 := {q2[2],q2[3],q2[4]};
    A3 := a1*A2 + a2*A1 + cross(A1, A2);
    q3 := {a1*a2 - A1*A2,A3[1],A3[2],A3[3]};
  end QMult;
  
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
  
  model OnePortBody
    extends RigidBody;
    WrenchPort InPort 
      annotation (Placement(transformation(extent={{-20,60},{20,100}})));
  equation
    F = InPort.F;
    M = InPort.M + cross(InPort.P - r, InPort.F);
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}),      graphics));
  end OnePortBody;
  
  model OnePortHeavyBody
    extends RigidBody;
    parameter SI.Acceleration[3] Gravity;
    WrenchPort InPort 
      annotation (Placement(transformation(extent={{-20,60},{20,100}})));
  equation
    F = m*Gravity + InPort.F;
    M = InPort.M + cross(InPort.P - r, InPort.F);
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}),      graphics));
  end OnePortHeavyBody;
  
  model TwoPortsHeavyBody
    extends RigidBody;
    parameter SI.Acceleration[3] Gravity;
    WrenchPort InPort 
      annotation (Placement(transformation(extent={{-60,59},{-20,99}})));
    WrenchPort InPort1 
      annotation (Placement(transformation(extent={{20,60},{60,100}})));
  equation
    F = m*Gravity + InPort.F + InPort1.F;
    M = InPort.M + cross(InPort.P - r, InPort.F) + InPort1.M + cross(InPort1.P - r, InPort1.F);
    annotation (Diagram(coordinateSystem(preserveAspectRatio=true,  extent={{-100,
              -100},{100,100}}),      graphics));
    connect(InPort, InPort) annotation (Line(
        points={{-40,79},{-40,79}},
        color={0,0,255},
        smooth=Smooth.None));
  end TwoPortsHeavyBody;
  
  model ThreePortsHeavyBody
    extends RigidBody;
    parameter SI.Acceleration[3] Gravity;
    WrenchPort InPort 
      annotation (Placement(transformation(extent={{-80,60},{-40,100}})));
    WrenchPort InPort1 
      annotation (Placement(transformation(extent={{-20,60},{20,100}})));
    WrenchPort InPort2 
      annotation (Placement(transformation(extent={{40,60},{80,100}})));
  equation
    F = m*Gravity + InPort.F + InPort1.F + InPort2.F;
    M = InPort.M + cross(InPort.P - r, InPort.F) + InPort1.M + cross(InPort1.P - r, InPort1.F) +
        InPort2.M + cross(InPort2.P - r, InPort2.F);
    annotation (Diagram(coordinateSystem(preserveAspectRatio=true,  extent={{-100,
              -100},{100,100}}),      graphics));
    connect(InPort, InPort) annotation (Line(
        points={{-60,80},{-60,80}},
        color={0,0,255},
        smooth=Smooth.None));
  end ThreePortsHeavyBody;
  
  model FourPortsHeavyBody
    extends RigidBody;
    parameter SI.Acceleration[3] Gravity;
    WrenchPort InPort 
      annotation (Placement(transformation(extent={{-86,60},{-46,100}})));
    WrenchPort InPort1 
      annotation (Placement(transformation(extent={{-42,60},{-2,100}})));
    WrenchPort InPort2 
      annotation (Placement(transformation(extent={{2,60},{42,100}})));
    WrenchPort InPort3 
      annotation (Placement(transformation(extent={{46,60},{86,100}})));
  //  Real MR;
  //  Real[3] d;
  equation
  //  d = cross({0, 1, 0}, OutPort.T*{0, 0, 1});
    F = m*Gravity + InPort.F + InPort1.F + InPort2.F + InPort3.F;
    M = InPort.M + cross(InPort.P - r, InPort.F) + InPort1.M + cross(InPort1.P - r, InPort1.F) +
        InPort2.M + cross(InPort2.P - r, InPort2.F) + InPort3.M + cross(InPort3.P - r, InPort3.F);
  // + MR*d/sqrt(d*d);
    annotation (Diagram(coordinateSystem(preserveAspectRatio=true,  extent={{-100,
              -100},{100,100}}),      graphics));
    connect(InPort, InPort) annotation (Line(
        points={{-66,80},{-66,80}},
        color={0,0,255},
        smooth=Smooth.None));
  end FourPortsHeavyBody;
  
  model FivePortsHeavyBody
    extends RigidBody;
    parameter SI.Acceleration[3] Gravity;
    WrenchPort InPort 
      annotation (Placement(transformation(extent={{-86,60},{-46,100}})));
    WrenchPort InPort1 
      annotation (Placement(transformation(extent={{-42,60},{-2,100}})));
    WrenchPort InPort2 
      annotation (Placement(transformation(extent={{2,60},{42,100}})));
    WrenchPort InPort3 
      annotation (Placement(transformation(extent={{46,60},{86,100}})));
    WrenchPort InPort4 
      annotation (Placement(transformation(extent={{60,-20},{100,20}})));
  equation
    F = m*Gravity + InPort.F + InPort1.F + InPort2.F + InPort3.F + InPort4.F;
    M = InPort.M + cross(InPort.P - r, InPort.F) + InPort1.M + cross(InPort1.P - r, InPort1.F) +
        InPort2.M + cross(InPort2.P - r, InPort2.F) + InPort3.M + cross(InPort3.P - r, InPort3.F) +
        InPort4.M + cross(InPort4.P - r, InPort4.F);
    annotation (Diagram(coordinateSystem(preserveAspectRatio=true,  extent={{-100,
              -100},{100,100}}),      graphics));
    connect(InPort, InPort) annotation (Line(
        points={{-66,80},{-66,80}},
        color={0,0,255},
        smooth=Smooth.None));
  end FivePortsHeavyBody;
  
  model RigidBodyEnergized
    extends RigidBody;
    SI.Energy Kt;
    SI.Energy Kr;
    SI.Energy K;
  equation
    Kt = 0.5*m*v*v;
    Kr = 0.5*I*omega*omega;
    K = Kt + Kr;
  end RigidBodyEnergized;
  
  partial model BaseBody
    replaceable KinematicPort OutPort annotation (extent=[-10,-100; 10,-80],
        Placement(transformation(extent={{-20,-100},{20,-60}})));
  
    annotation (Icon, Diagram(coordinateSystem(preserveAspectRatio=false,
            extent={{-100,-100},{100,100}}), graphics));
  end BaseBody;
  
  model Base
    extends BaseBody;
  /*  VisualShape Plane(
    r0={0,0,0},
    Shape="box",
    LengthDirection={0,-1,0},
    WidthDirection={1,0,0},
    Length=0.1,
    Width=10,
    Height=10,
    Material={0,0,1,0});*/
  equation
  
    OutPort.r = {0,0,0};
    OutPort.v = {0,0,0};
    OutPort.a = {0,0,0};
    OutPort.T = [1, 0, 0; 0, 1, 0; 0, 0, 1];
    OutPort.omega = {0,0,0};
    OutPort.epsilon = {0,0,0};
  /*  Plane.S = OutPort.T;
  Plane.r = OutPort.r;*/
    annotation (Icon(
        Polygon(points=[-100,-40; -60,0; 100,0; 60,-40; -100,-40],      style(
              pattern=0, fillPattern=1)),
        Line(points=[-100,-40; -60,0],   style(color=0, fillPattern=1)),
        Line(points=[-60,0; 100,0],   style(color=0, fillPattern=1)),
        Polygon(points=[60,-40; 100,0; 100,-20; 60,-60; 60,-40],      style(
              pattern=0, fillPattern=1)),
        Rectangle(extent=[-100,-40; 60,-60],   style(color=0, fillPattern=1)),
        Line(points=[60,-40; 100,0],   style(color=0, fillPattern=1)),
        Line(points=[100,0; 100,-20],   style(color=0, fillPattern=1)),
        Line(points=[60,-60; 100,-20],   style(color=0, fillPattern=1))),
        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
              {100,100}}), graphics));
  end Base;
  
  partial model Constraint
    parameter Integer ConstraintNo=1;
    replaceable KinematicPort InPortA annotation (extent=[-40,-100; -20,-80],
        Placement(transformation(extent={{-60,-100},{-20,-60}})));
    replaceable WrenchPort OutPortA annotation (extent=[-40,80; -20,100],
        Placement(transformation(extent={{-60,60},{-20,100}})));
    replaceable KinematicPort InPortB annotation (extent=[20,-100; 40,-80],
        Placement(transformation(extent={{20,-100},{60,-60}})));
    replaceable WrenchPort OutPortB annotation (extent=[20,80; 40,100],
        Placement(transformation(extent={{20,60},{60,100}})));
    annotation (Icon, Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,
              -100},{100,100}}),       graphics));
  equation
    OutPortA.F + OutPortB.F = zeros(3);
    OutPortA.M + OutPortB.M = zeros(3);
  end Constraint;
  
  partial model ContactTracking
    extends Constraint;
    SI.Position[3] rA;
    SI.Position[3] rB;
    Real mu;
    Real lambda;
    Real[3] gradfA;
    Real[3] gradfB;
    Real[3] gradgA;
    Real[3] gradgB;
    Real[3] nA;
    Real[3] rhoA(start = {0, 0, 0});
    Real[3] rhoB(start = {0, 0, 0});
  equation
    gradgB = lambda*gradgA;
    rB - rA = mu*gradgA;
    rA = InPortA.r + InPortA.T*rhoA;
    rB = InPortB.r + InPortB.T*rhoB;
    gradgA = InPortA.T*gradfA;
    gradgB = InPortB.T*gradfB;
    nA = gradgA/sqrt(gradgA*gradgA);
  end ContactTracking;
  
  partial model PointContact
    extends ContactTracking;
    Real[3] vA;
    Real[3] vB;
    Real[3] vr;
    Real vrt;
    Real[3] RBt;
    Real RBn;
  equation
    mu = 0;
    vA = InPortA.v + cross(InPortA.omega, rA - InPortA.r);
    vB = InPortB.v + cross(InPortB.omega, rB - InPortB.r);
    vr = vB - vA;
    vrt = sqrt(vr*vr);
    RBn = OutPortB.F*nA;
    OutPortB.F = RBt + RBn*nA;
    OutPortA.P = rA;
    OutPortB.P = rB;
  end PointContact;
  
  partial model SlidingWithDryFriction
    extends PointContact;
    parameter Real delta = 10^(-6);
    parameter Real fric = 0.1;
    Real kappa;
  equation
    RBt = -fric*vr*(if vrt <= delta then 1/delta else 1/vrt) + kappa*nA; // N is absent
    OutPortB.M = zeros(3);
  end SlidingWithDryFriction;
  
  partial model Rolling
    extends PointContact;
    parameter Real delta = 10^(-6);
    parameter Real fric = 0.1;
    Real kappa;
  equation
    vr = kappa*nA;
  end Rolling;
  
  model Rigid
    extends Constraint;
    parameter SI.Position[3] rA = zeros(3) "Constraint position in body A";
    parameter SI.Position[3] rB = zeros(3) "Constraint position in body B";
    SI.Position[3] RA;
    SI.Position[3] RB;
    SI.Velocity[3] vA;
    SI.Velocity[3] vB;
  equation
    RA = InPortA.r + InPortA.T*rA;
    RB = InPortB.r + InPortB.T*rB;
    vA = InPortA.v + cross(InPortA.omega, InPortA.T*rA);
    vB = InPortB.v + cross(InPortB.omega, InPortB.T*rB);
    vA = vB;
    InPortA.epsilon = InPortB.epsilon;
    OutPortA.P = RA;
    OutPortB.P = RB;
    annotation (Icon(
        Rectangle(extent=[-80,40; 100,-40],style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-100,-20; -80,0],
                                     style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-100,0; -80,20], style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-100,-40; -80,-20],style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-100,-60; -80,-40],style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-60,-60; -40,-40], style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-80,-60; -60,-40], style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-100,-50; -80,-30],style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-50,-60; -40,-50], style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-70,-60; -50,-40], style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-90,-60; -70,-40], style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-100,20; -80,40],  style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-100,-10; -80,10], style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-100,-30; -80,-10],style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-100,10; -80,30],  style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-80,40; -60,60],   style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-60,40; -40,60],   style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-50,40; -40,50],   style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-70,40; -50,60],   style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-100,30; -70,60],  style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-100,40; -80,60],  style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-100,50; -90,60],  style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Rectangle(extent=[-100,100; 100,-100], style(pattern=0)),
        Text(
          extent=[-34,24; 52,-20],
          style(color=3, rgbcolor={0,0,255}),
          string="%name")),                Diagram);
  end Rigid;
  
  partial model Joint
    extends Constraint;
    parameter Real[3] nA;
    parameter Real[3] nB;
    // Redundant parameter
    parameter SI.Position[3] rA "Joint position in body A";
    parameter SI.Position[3] rB "Joint position in body B";
    // Radius-vector of the joint point in Body A & Body B
    SI.Position[3] RA;
    SI.Position[3] RB;
    SI.Velocity[3] vBa;
    // Absolute velocity
    SI.Velocity[3] vBe;
    // Convective velocity
    SI.Velocity[3] vBr(start = zeros(3));
    // Relative velocity
    SI.Acceleration[3] aBa;
    // Absolute acceleration
    SI.Acceleration[3] aBe;
    // Convective acceleration
    SI.Acceleration[3] aBr;
    // Relative acceleration
    SI.AngularVelocity[3] omegar;
    // Relative angular velocity
    SI.AngularAcceleration[3] epsilonr;
    // Relative angular acceleration
    Real nAi[3];
    // Unit vector of joint axis w. r. t. inertial frame
    SI.Force F;
    // Force along joint axis
    SI.Torque M;
    // Torque about joint axis
    SI.Acceleration mu;
    // Acceleration along joint axis
    //  SI.AngularVelocity lambda; // Angular acceleration about joint axis
    SI.AngularAcceleration lambda;
  //  SI.Velocity mu;
  equation
    RA = InPortA.r + InPortA.T*rA;
    RB = InPortB.r + InPortB.T*rB;
  
    nAi = InPortA.T*nA;
  
    vBa = InPortB.v + cross(InPortB.omega, InPortB.T*rB);
    vBe = InPortA.v + cross(InPortA.omega, RB - InPortA.r);
    vBa = vBe + vBr;
  //  vBr = mu*nAi;
  
    aBa = InPortB.a + cross(InPortB.epsilon, InPortB.T*rB) + cross(InPortB.
      omega, cross(InPortB.omega, InPortB.T*rB));
    aBe = InPortA.a + cross(InPortA.epsilon, RB - InPortA.r) + cross(InPortA.
      omega, cross(InPortA.omega, RB - InPortA.r));
    aBa = aBe + 2*cross(InPortA.omega, vBr) + aBr;
    aBr = mu*nAi;
  
  //  aBr = der(vBr);
  
    omegar = InPortB.omega - InPortA.omega;
    //  omegar = lambda*nAi;
    epsilonr = InPortB.epsilon - InPortA.epsilon - cross(InPortA.omega, omegar);
    epsilonr = lambda*nAi;
  
    F = OutPortA.F*nAi;
    M = OutPortA.M*nAi;
  
    OutPortA.P = RA;
    OutPortB.P = RA;
  
    // Translatory alternatives:
    //  mu = 0; // Fixed joint
    //  F = 0; // Free joint
    //  mu = ...; // Translatory compliance
  
    // Rotary alternatives:
    //  lambda = 0; // Only translatory joint
    //  M = 0; // Ideal joint
    //  lambda = ...; // Control by kinematics
    //  M = ...; // Control by torque
  
    annotation (Diagram, Icon(
        Rectangle(extent=[-52,40; 100,-40],style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-100,-20; -80,0],
                                     style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-100,0; -80,20], style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-100,-40; -80,-20],style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-100,-70; -80,-50],style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-60,-70; -40,-50], style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-80,-70; -60,-50], style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-100,-50; -80,-30],style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-70,-70; -50,-50], style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-90,-70; -70,-50], style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-100,20; -80,40],  style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-100,-10; -80,10], style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-100,-30; -80,-10],style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-100,10; -80,30],  style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-80,50; -60,70],   style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-60,50; -40,70],   style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-70,50; -50,70],   style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-100,40; -70,70],  style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-100,50; -80,70],  style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-100,60; -90,70],  style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-80,50; 0,50],     style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-80,-50; 0,-50],   style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-100,30; -80,50],  style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-100,-60; -80,-40],style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-80,50; -80,-50],  style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-40,50; -20,70],   style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-20,50; 0,70],     style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-10,50; 0,60],     style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-30,50; -10,70],   style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-50,50; -30,70],   style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-30,-70; -10,-50], style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-50,-70; -30,-50], style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-10,-70; 0,-60],   style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-40,-70; -20,-50], style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-20,-70; 0,-50],   style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Rectangle(extent=[-100,100; 100,-100], style(pattern=0))));
  end Joint;
  
  model FixedJoint
    extends Joint;
  equation
    mu = 0;
    M = 0;
    // Ideal joint
    annotation (Icon(
        Text(
          extent=[-50,36; 96,-38],
          style(color=3, rgbcolor={0,0,255}),
          string="%name")));
  end FixedJoint;
  
  model ResistFixedJoint
    extends Joint;
    parameter Stiffness c=1000 "Stiffness";
    parameter Viscosity d=1000 "Viscosity";
    SI.Angle phi(start = 0);
    SI.AngularVelocity dphi(start = 0);
  equation
    der(phi) = dphi;
    der(dphi) = lambda;
    mu = 0;
    M = c*phi + d*dphi;
    annotation (Icon(
        Text(
          extent=[-52,26; 100,-24],
          style(color=3, rgbcolor={0,0,255}),
          string="ResistFixedJoint"),
        Line(points=[-80,-20; -70,-20], style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-56,-20; -52,-20], style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Rectangle(extent=[-76,-10; -56,-30], style(color=0, rgbcolor={0,0,0})),
        Rectangle(extent=[-66,-10; -56,-30], style(
            color=0,
            rgbcolor={0,0,0},
            fillColor=10,
            rgbfillColor={135,135,135})),
        Rectangle(extent=[-70,-10; -66,-30], style(
            color=0,
            rgbcolor={0,0,0},
            fillColor=0,
            rgbfillColor={0,0,0})),
        Line(points=[-80,20; -76,20; -72,10; -66,30; -60,10; -56,20; -52,20],
            style(
            color=0,
            rgbcolor={0,0,0},
            fillPattern=1))));
  
  end ResistFixedJoint;
  
  model FreeJoint
    extends Joint;
  equation
    F = 0;
    M = 0;
    // Ideal joint
    annotation (Icon(Text(
          extent=[-50,36; 98,-38],
          style(color=3, rgbcolor={0,0,255}),
          string="FreeJoint")));
  end FreeJoint;
  
  model SpringJoint
    extends Joint;
    parameter Real c=1000 "Stiffness";
    parameter Real d=1000 "Viscosity";
    SI.Position nu(start=0);
    SI.Velocity dnu(start=0);
    SI.Work Aelastic;
    SI.Work Aviscos;
    SI.Energy Ed;
  equation
    der(Aelastic) = -c*nu*dnu;
    der(Aviscos) = -d*dnu^2;
    Ed = Aviscos;
    //  der(nu) = dnu;
    //  OutPortB.F*nAi = -c*nu - d*dnu;
    //  (RB - RA)*nAi = nu;
    der(nu) = dnu;
    der(dnu) = mu;
    F = c*nu + d*dnu;
    M = 0;
    // Ideal joint
    annotation (Icon(
        Text(
          extent=[-52,34; 100,-32],
          style(color=3, rgbcolor={0,0,255}),
          string="SpringJoint"),
        Line(points=[-80,20; -76,20; -72,10; -66,30; -60,10; -56,20; -52,20],
            style(
            color=0,
            rgbcolor={0,0,0},
            fillPattern=1)),
        Line(points=[-80,-20; -70,-20], style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-56,-20; -52,-20], style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Rectangle(extent=[-76,-10; -56,-30], style(color=0, rgbcolor={0,0,0})),
        Rectangle(extent=[-66,-10; -56,-30], style(
            color=0,
            rgbcolor={0,0,0},
            fillColor=10,
            rgbfillColor={135,135,135})),
        Rectangle(extent=[-70,-10; -66,-30], style(
            color=0,
            rgbcolor={0,0,0},
            fillColor=0,
            rgbfillColor={0,0,0}))));
  
  end SpringJoint;
  
  model FixedServoJoint
    extends Joint;
    input SI.AngularAcceleration Control;
    // Control
    SI.Angle phi(start=0);
    SI.AngularVelocity dphi(start=0);
  initial equation
    //  omegar = dphi*nAi;
  equation
    der(phi) = dphi;
    //  dphi = omegar*nAi;
    der(dphi) = lambda;
    //  der(phi) = lambda;
    lambda = Control;
    mu = 0;
    // Fixed joint
    annotation (Icon(Text(
          extent=[-52,34; 100,-32],
          style(color=3, rgbcolor={0,0,255}),
          string="FixedServoJoint")));
  end FixedServoJoint;
  
  model PrismaticServoJoint
    extends Joint;
    input SI.Acceleration Control;
    SI.Position shift(start = 0);
    SI.Velocity dshift(start = 0);
  equation
    der(shift) = dshift;
    der(dshift) = mu;
    lambda = 0;
    mu = Control;
    annotation (Icon(
        Text(
          extent=[-48,36; 98,-38],
          style(color=3, rgbcolor={0,0,255}),
          string="%name")));
  end PrismaticServoJoint;
  
  model SpringPrismatic
    extends Joint;
    parameter Stiffness c = 1000 "Stiffness";
    parameter Viscosity d = 1000 "Viscosity";
    SI.Position nu(start = 0);
    SI.Velocity dnu(start = 0);
  equation
    der(nu) = dnu;
    der(dnu) = mu;
    F = c*nu + d*dnu;
    lambda = 0;
    annotation (Icon(
        Text(
          extent=[-52,34; 100,-32],
          style(color=3, rgbcolor={0,0,255}),
          string="SpringPrismatic"),
        Line(points=[-80,20; -76,20; -72,10; -66,30; -60,10; -56,20; -52,20],
            style(
            color=0,
            rgbcolor={0,0,0},
            fillPattern=1)),
        Line(points=[-80,-20; -70,-20], style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Line(points=[-56,-20; -52,-20], style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=10,
            rgbfillColor={135,135,135})),
        Rectangle(extent=[-76,-10; -56,-30], style(color=0, rgbcolor={0,0,0})),
        Rectangle(extent=[-66,-10; -56,-30], style(
            color=0,
            rgbcolor={0,0,0},
            fillColor=10,
            rgbfillColor={135,135,135})),
        Rectangle(extent=[-70,-10; -66,-30], style(
            color=0,
            rgbcolor={0,0,0},
            fillColor=0,
            rgbfillColor={0,0,0}))));
  
  end SpringPrismatic;
  
  model Touch
    extends Constraint;
    parameter SI.Position[3] rA "Position in body A cage";
    parameter SI.Position[3] rB "Position in body B ball";
    // Radius-vector of the joint point in Body A & Body B
    SI.Position[3] RA;
    SI.Position[3] RB;
    SI.Velocity[3] vBa;
    // Absolute velocity
    SI.Velocity[3] vBe;
    // Convective velocity
    SI.Velocity[3] vBr;
    // Relative velocity
    SI.Acceleration[3] aBa;
    // Absolute acceleration
    SI.Acceleration[3] aBe;
    // Convective acceleration
    SI.Acceleration[3] aBr "Relative acceleration";
    Real nAi[3] "Unit vector of ball shift w. r. t. inertial frame";
    SI.Acceleration mu;
    // Acceleration along joint axis
  equation
    RA = InPortA.r + InPortA.T*rA;
    RB = InPortB.r + InPortB.T*rB;
  
    nAi = InPortA.T*rA;
  
    vBa = InPortB.v + cross(InPortB.omega, InPortB.T*rB);
    vBe = InPortA.v + cross(InPortA.omega, RB - InPortA.r);
    vBa = vBe + vBr;
  
    aBa = InPortB.a + cross(InPortB.epsilon, InPortB.T*rB) + cross(InPortB.
      omega, cross(InPortB.omega, InPortB.T*rB));
    aBe = InPortA.a + cross(InPortA.epsilon, RB - InPortA.r) + cross(InPortA.
      omega, cross(InPortA.omega, RB - InPortA.r));
    aBa = aBe + 2*cross(InPortA.omega, vBr) + aBr;
    aBr = mu*nAi;
  
  //  aBr = der(vBr);
  
    0 = OutPortA.F*nAi;
    zeros(3) = OutPortA.M;
  
    OutPortA.P = RA;
    OutPortB.P = RA;
  
    // Translatory alternatives:
    //  mu = 0; // Fixed joint
    //  F = 0; // Free joint
    //  mu = ...; // Translatory compliance
  
    // Rotary alternatives:
    //  lambda = 0; // Only translatory joint
    //  M = 0; // Ideal joint
    //  lambda = ...; // Control by kinematics
    //  M = ...; // Control by torque
  
    annotation (Diagram, Icon(
        Rectangle(extent=[-100,100; 100,-100], style(color=3, rgbcolor={0,0,255})),
        Line(points=[-26,38; -18,30; -12,22; -8,16; -4,10; -2,4; 0,-4; 0,-10; 0,-16; -2,
              -26; -6,-34; -10,-42; -14,-48; -16,-52; -22,-58; -26,-62], style(
            color=0,
            rgbcolor={0,0,0},
            fillColor=1,
            rgbfillColor={255,0,0},
            fillPattern=1)),
        Line(points=[22,40; 16,34; 10,26; 6,20; 2,10; 0,2; 0,-2; 0,-8; 0,-16; 2,
              -26; 6,-36; 10,-44; 14,-50; 18,-56; 22,-60], style(
            color=0,
            rgbcolor={0,0,0},
            fillColor=1,
            rgbfillColor={255,0,0},
            fillPattern=1)),
        Text(
          extent=[-44,82; 42,38],
          style(color=3, rgbcolor={0,0,255}),
          string="%name")));
  end Touch;
  
  model Spring
    extends Constraint;
    parameter SI.Length l "undeformed spring length";
    parameter Real c "spring stiffness";
    parameter SI.Position[3] rA "fixed point on Body A";
    parameter SI.Position[3] rB "fixed point on Body B";
    SI.Position[3] RA "fixed point on Body A";
    SI.Position[3] RB "fixed point on Body B";
    SI.Length[3] RAB;
    SI.Length deltal;
  equation
    RA = InPortA.r + InPortA.T*rA;
    RB = InPortB.r + InPortB.T*rB;
    OutPortA.P = RA;
    OutPortB.P = RB;
    RAB = RB - RA;
    deltal = sqrt((RAB - l*RAB/sqrt(RAB*RAB))*(RAB - l*RAB/sqrt(RAB*RAB)));
    OutPortB.F = -c*deltal*RAB/sqrt(RAB*RAB);
    OutPortB.M = zeros(3);
    annotation (Icon(         Line(points=[0,58; 0,40; 20,30; -20,10; 20,-10;
              -20,-30; 0,-40; 0,-60],              style(color=0,
              rgbcolor={0,0,0})), Rectangle(extent=[-100,100; 100,-100], style(
              color=3, rgbcolor={0,0,255})),
        Text(
          extent=[-80,40; 80,-40],
          style(color=3, rgbcolor={0,0,255}),
          string="%name")));
  end Spring;
  
  model NPortsHeavyBody
    extends RigidBody;
  
    parameter SI.Acceleration[3] Gravity;
    parameter Integer NPorts = 1;
    WrenchPort[NPorts] InPorts;
  equation
    F = m*Gravity + sum(InPorts[i].F for i in 1:NPorts);
    M = sum(InPorts[i].M + cross(InPorts[i].P - r, InPorts[i].F) for i in 1:NPorts);
  
  end NPortsHeavyBody;
  
  model ForceBase
                  extends Base;
    WrenchPort InPort;
  equation
    InPort.P = zeros(3);
    InPort.F = zeros(3);
    InPort.M = zeros(3);
  end ForceBase;
  
  function QMult1 // why did it have this ----> "No protected - for array construction" ???
    input Real q1[4];
    input Real q2[4];
    output Real q3[4];
  protected
    Real tmp[3];
  algorithm
    tmp := q1[1]*{q2[2],q2[3],q2[4]} + q2[1]*{q1[2],q1[3],q1[4]} + cross({q1[2],q1[3],q1[4]},{q2[2],q2[3],q2[4]});
    q3 := { q1[1]*q2[1] - {q1[2],q1[3],q1[4]}*{q2[2],q2[3],q2[4]}
          , tmp[1]
          , tmp[2]
          , tmp[3]
	  };
    annotation(Inline=true);
  end QMult1;
  
  function QToT "Converts quaternion to Rotation matrix."
    input Real[4] q;
    output Real[3,3] T;
  
  algorithm
    T := [ q[1]^2 + q[2]^2 - q[3]^2 - q[4]^2
         , 2*(q[2]*q[3] - q[1]*q[4])
         , 2*(q[2]*q[4] + q[1]*q[3])

         ; 2*(q[1]*q[4] + q[2]*q[3])
         , q[1]^2 - q[2]^2 + q[3]^2 - q[4]^2
         , 2*(q[3]*q[4] - q[1]*q[2])

         ; 2*(q[2]*q[4] - q[1]*q[3])
         , 2*(q[1]*q[2] + q[3]*q[4])
         , q[1]^2 - q[2]^2 - q[3]^2 + q[4]^2
         ] / (q*q);
    annotation(Inline=true);
  end QToT;

end Basics;
