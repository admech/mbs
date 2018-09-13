within ;
package ThreeD_MBS_Dynamics

  package Icons "Icon definitions"
  extends Modelica.Icons.Library2;
  annotation (Icon(Text(
        extent=[-59, -9; 42, -56],
        string="icons",
        style(color=0))));

  partial model FunctionIcon "Icon for a function"
    annotation (Icon(
        Text(extent=[-134, 104; 142, 44], string="%name"),
        Ellipse(extent=[-100, 40; 100, -100], style(color=45, fillColor=7)),
        Text(
          extent=[-84, -4; 84, -52],
          string="function",
          style(color=45))));
  equation

  end FunctionIcon;

  partial model NormalForceIcon

    annotation (Diagram, Icon(
        Rectangle(extent=[-100, 0; 100, -68], style(gradient=1)),
        Ellipse(extent=[-84, 68; 82, -34], style(color=0, fillColor=7)),
        Line(points=[-100, 0; 80, 0], style(
            color=0,
            fillColor=7,
            fillPattern=1)),
        Rectangle(extent=[-100,100; 100,-100], style(pattern=0))));

  end NormalForceIcon;
  end Icons;
extends Modelica.Icons.Library;

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

  package Examples

    package Rattleback
      model Base
        extends BaseBody;
        VisualShape Plane(
          r0={0,0,0},
          Shape="box",
          LengthDirection={0,-1,0},
          WidthDirection={1,0,0},
          Length=0.1,
          Width=10,
          Height=10,
          Material={0,0,1,0});
        WrenchPort InPortRoll annotation (extent=[-10, 80; 10, 100]);
        annotation (Diagram);
      equation
        OutPort.r = {0,0,0};
        OutPort.v = {0,0,0};
        OutPort.a = {0,0,0};
        OutPort.T = [1, 0, 0; 0, 1, 0; 0, 0, 1];
        OutPort.omega = {0,0,0};
        OutPort.epsilon = {0,0,0};
        Plane.S = OutPort.T;
        Plane.r = OutPort.r;
      end Base;

      model Case_of_Kane_Rattleback
        extends RigidBody;
        outer Real[3] Gravity;
        // Ellipsoid semi-diameters
        outer Real a1;
        outer Real b1;
        outer Real c1;
        outer Real h;
        outer SI.Angle delta;
        parameter Real R[3, 3]=[cos(delta), 0, sin(delta); 0, 1, 0; -sin(delta), 0,
             cos(delta)];
        VisualShape Body(
          r0={0,h,-c1},
          Shape="sphere",
          LengthDirection={0,0,1},
          WidthDirection={1,0,0},
          Length=2*c1,
          Width=2*a1,
          Height=2*b1,
          Material={1,0,0,1});
        //  Real p1[3];
        Real E;
        Real K1;
        Real K2;
        Real P;
        Real Mod_q2;
        Real gamma(start=0);
        Real KaneGamDeg;
        WrenchPort InPortRoll annotation (extent=[-10, 80; 10, 100]);
      equation
        F = m*Gravity + InPortRoll.F;
        M = InPortRoll.M + cross(InPortRoll.P - r, InPortRoll.F);
        Body.S = T*R;
        Body.r = r;
        //  Body.r = {0,r[2],r[3]};
        //  p1 = r + T*{2,0,0};
        K1 = 0.5*m*v*v;
        K2 = 0.5*omega*I*omega;
        P = -m*r*Gravity;
        E = K1 + K2 + P;
        Mod_q2 = q[1]^2 + q[2]^2 + q[3]^2 + q[4]^2;
        der(gamma) = OutPort.omega[2];
        KaneGamDeg = -gamma*180/Modelica.Constants.pi;
        annotation (Diagram);
      end Case_of_Kane_Rattleback;

      model Case_of_Kane_Constraint
        extends Roll;
        // Ellipsoid semiaxes
        outer Real a1;
        outer Real b1;
        outer Real c1;
        outer Real delta;
        outer Real h;

          // Matrix of rotation with an angle delta about y-axis relative to principal-axis system
        parameter Real R[3, 3]=[cos(delta), 0, sin(delta); 0, 1, 0; -sin(delta), 0,
             cos(delta)];
        // Matrix of ellipsoid equation relative to canonic axes
        parameter Real B1[3, 3]=[1/a1^2, 0, 0; 0, 1/b1^2, 0; 0, 0, 1/c1^2];
        // Matrix of ellipsoid equation relative to principal axes
        parameter Real B[3, 3]=R*B1*transpose(R);
        // Normal vector to plane
        parameter Real n[3]={0,1,0};
        // Shift of a surface center
        parameter Real x0[3]={0,h,0};
        parameter Real d=0;
        Real lambda;
      equation
        n*r = d;
        n = lambda*InPortB.T*B*(transpose(InPortB.T)*(r - InPortB.r) - x0);
      end Case_of_Kane_Constraint;

      model Case_of_Kane_Test
        parameter Real g=9.81;
        parameter Real alpha=0.5*Modelica.Constants.pi/180;
        parameter Real beta=0.5*Modelica.Constants.pi/180;
        inner parameter Real[3] Gravity={0,-g,0};

        // Ellipsoid semi-diameters
        inner parameter Real a1=0.2;
        inner parameter Real b1=0.02;
        inner parameter Real c1=0.03;
        inner parameter Real delta=0.01428182891938;
        parameter Real qalpha[4]={cos(alpha/2),sin(alpha/2)*cos(delta),0,-sin(alpha
            /2)*sin(delta)};
        parameter Real qbeta[4]={cos(beta/2),sin(beta/2)*sin(delta),0,sin(beta/2)*
            cos(delta)};
        inner parameter Real h=0.01;
        annotation (Diagram);
        Base Base1(Plane(
            Length=0.01,
            Width=0.5,
            Height=0.5)) annotation (extent=[-60, 0; -40, 20]);
        Case_of_Kane_Rattleback RollingBody1(
          q(start=QMult(qalpha, QMult(qbeta, {1,0,0,0}))),
          r(start={0.0172989203771403,0.0100769534556172,-0.000303278424757918}),
          I=[0.000199714344, 0, 0; 0, 0.0017, 0; 0, 0, 0.001600285656],
          v(start={-0.0503696159308381,0.086491308436296,0.000754798213773296}),
          omega(start={0,5,0})) annotation (extent=[20, 0; 40, 20]);
        Case_of_Kane_Constraint Ellipsoid_on_Plane1 
          annotation (extent=[-20, 0; 0, 20]);
      equation
        connect(Base1.InPortRoll, Ellipsoid_on_Plane1.OutPortA) 
          annotation (points=[-50, 19; -50, 30; -13, 30; -13, 19]);
        connect(Base1.OutPort, Ellipsoid_on_Plane1.InPortA) 
          annotation (points=[-50, 1; -50, -10; -13, -10; -13, 1]);
        connect(Ellipsoid_on_Plane1.InPortB, RollingBody1.OutPort) 
          annotation (points=[-7, 1; -7, -10; 30, -10; 30, 1]);
        connect(Ellipsoid_on_Plane1.OutPortB, RollingBody1.InPortRoll) 
          annotation (points=[-7, 19; -7, 30; 30, 30; 30, 19]);
      end Case_of_Kane_Test;

      model RollingBody
        extends RigidBody;
        outer SI.Acceleration[3] Gravity;
        // Ellipsoid semiaxes
        outer SI.Length a1;
        outer SI.Length b1;
        outer SI.Length c1;
        outer SI.Angle delta;
        parameter Real R[3, 3]=[cos(delta), 0, sin(delta); 0, 1, 0; -sin(delta), 0,
             cos(delta)];
        VisualShape Body(
          r0={0,0,-c1},
          Shape="sphere",
          LengthDirection={0,0,1},
          WidthDirection={1,0,0},
          Length=2*c1,
          Width=2*a1,
          Height=2*b1,
          Material={1,0,0,1});
        SI.Position p1[3];
        SI.Energy E;
        SI.Energy K1;
        SI.Energy K2;
        SI.Energy P;
        Real Mod_q2;
        SI.Force Ft;
        WrenchPort InPortRoll annotation (extent=[-10, 80; 10, 100]);
      equation
        F = m*Gravity + InPortRoll.F;
        M = InPortRoll.M + cross(InPortRoll.P - r, InPortRoll.F);
        Body.S = T*R;
        Body.r = r;
        //  Body.r = {0,r[2],r[3]};
        p1 = r + T*{2,0,0};
        K1 = 0.5*m*v*v;
        K2 = 0.5*omega*I*omega;
        P = -m*r*Gravity;
        E = K1 + K2 + P;
        Mod_q2 = q[1]^2 + q[2]^2 + q[3]^2 + q[4]^2;
        Ft = sqrt(InPortRoll.F[1]^2 + InPortRoll.F[3]^2);
        annotation (Diagram);
      end RollingBody;

      model Ellipsoid_on_Plane
        extends Roll;
        // Ellipsoid semi-diameters
        outer SI.Length a1;
        outer SI.Length b1;
        outer SI.Length c1;
        outer SI.Angle delta;

          // Matrix of rotation with an angle delta about y-axis relative to principal-axis system
        parameter Real R[3, 3]=[cos(delta), 0, sin(delta); 0, 1, 0; -sin(delta), 0,
             cos(delta)];
        // Matrix of ellipsoid equation relative to canonic axes
        parameter Real B1[3, 3]=[1/a1^2, 0, 0; 0, 1/b1^2, 0; 0, 0, 1/c1^2];
        // Matrix of ellipsoid equation relative to principal axes
        parameter Real B[3, 3]=R*B1*transpose(R);
        // Normal vector to plane
        parameter Real n[3]={0,1,0};
        parameter Real d=0;
        SI.Velocity verr;
        SI.Velocity[3] verror;
        Real lambda;
      equation
        n*r = d;
        n = lambda*InPortB.T*B*transpose(InPortB.T)*(r - InPortB.r);
        verror = InPortA.v + cross(InPortA.omega, r - InPortA.r) - InPortB.v -
          cross(InPortB.omega, r - InPortB.r);
        verr = sqrt(verror*verror);
      end Ellipsoid_on_Plane;

      model Case_of_Borisov_Test
        parameter SI.Acceleration g=100;
        inner parameter SI.Acceleration[3] Gravity={0,-g,0};
        // Ellipsoid semi-diameters
        inner parameter SI.Length a1=2;
        inner parameter SI.Length b1=1;
        inner parameter SI.Length c1=3;
        inner parameter SI.Length delta=-0.2;
        annotation (
          Diagram,
          experiment(
            StopTime=500,
            NumberOfIntervals=100000,
            Tolerance=1e-010),
          experimentSetupOutput);
        Base Base1 annotation (extent=[-60, 0; -40, 20]);
        RollingBody RollingBody1(
          q(start={1,0,0,0}),
          r(start={0,1,0}),
          I=[6, 0, 0; 0, 7, 0; 0, 0, 5],
          v(start={2,0,0}),
          omega(start={0,19,-2})) annotation (extent=[20, 0; 40, 20]);
        Ellipsoid_on_Plane Ellipsoid_on_Plane1 annotation (extent=[-20, 0; 0, 20]);
      equation
        connect(Base1.InPortRoll, Ellipsoid_on_Plane1.OutPortA) 
          annotation (points=[-50, 19; -50, 30; -13, 30; -13, 19]);
        connect(Base1.OutPort, Ellipsoid_on_Plane1.InPortA) 
          annotation (points=[-50, 1; -50, -10; -13, -10; -13, 1]);
        connect(Ellipsoid_on_Plane1.InPortB, RollingBody1.OutPort) 
          annotation (points=[-7, 1; -7, -10; 30, -10; 30, 1]);
        connect(Ellipsoid_on_Plane1.OutPortB, RollingBody1.InPortRoll) 
          annotation (points=[-7, 19; -7, 30; 30, 30; 30, 19]);
      end Case_of_Borisov_Test;

      model Test
        parameter SI.Acceleration g=9.81;
        inner parameter SI.Acceleration[3] Gravity={0,-g,0};
        // Ellipsoid semi-diameters
        inner parameter SI.Length a1=2;
        inner parameter SI.Length b1=1;
        inner parameter SI.Length c1=3;
        inner parameter SI.Angle delta=Modelica.Constants.pi/10;
        annotation (Diagram, experiment(
            StopTime=100,
            NumberOfIntervals=50000,
            Tolerance=1e-006));
        Base Base1 annotation (extent=[-60, 0; -40, 20]);
        RollingBody RollingBody1(
          q(start={1,0,0,0}),
          r(start={0,1,0}),
          I=[2, 0, 0; 0, 3, 0; 0, 0, 1],
          v(start={0.05,0,0}),
          omega(start={0,-1,-0.05})) annotation (extent=[20, 0; 40, 20]);
        Ellipsoid_on_Plane Ellipsoid_on_Plane1 annotation (extent=[-20, 0; 0, 20]);
      equation
        connect(Base1.InPortRoll, Ellipsoid_on_Plane1.OutPortA) 
          annotation (points=[-50, 19; -50, 30; -13, 30; -13, 19]);
        connect(Base1.OutPort, Ellipsoid_on_Plane1.InPortA) 
          annotation (points=[-50, 1; -50, -10; -13, -10; -13, 1]);
        connect(Ellipsoid_on_Plane1.InPortB, RollingBody1.OutPort) 
          annotation (points=[-7, 1; -7, -10; 30, -10; 30, 1]);
        connect(Ellipsoid_on_Plane1.OutPortB, RollingBody1.InPortRoll) 
          annotation (points=[-7, 19; -7, 30; 30, 30; 30, 19]);
      end Test;
    end Rattleback;

    package RollingDisc
      model Base
        extends BaseBody;
        VisualShape Plane(
          r0={0,0,0},
          Shape="box",
          LengthDirection={0,-1,0},
          WidthDirection={0,0,1},
          Length=0.002,
          Width=20,
          Height=20,
          Material={0.4,0.6,0,0});
      //    Length=0.1
        annotation (Diagram);
      equation
        OutPort.r = {0,0,0};
        OutPort.v = {0,0,0};
        OutPort.a = {0,0,0};
        OutPort.T = [1, 0, 0; 0, 1, 0; 0, 0, 1];
        OutPort.omega = {0,0,0};
        OutPort.epsilon = {0,0,0};
        Plane.S = OutPort.T;
        Plane.r = OutPort.r;
      end Base;

      model RollingDisc
        extends RigidBody;
        parameter SI.Acceleration[3] Gravity;
        // Disc radius
        parameter SI.Radius r1;
        VisualShape Disc(
          r0={0,0,-0.005},
          Shape="cylinder",
          LengthDirection={0,0,1},
          WidthDirection={1,0,0},
          Length=0.01,
          Width=2*r1,
          Height=2*r1,
          Material={0.5,0.8,1,0.2});
        VisualShape Rod(
          r0={0,0,0.02},
          Shape="box",
          LengthDirection={0,-1,0},
          WidthDirection={0,0,1},
          Length=r1 - 0.05,
          Width=0.02,
          Height=0.05,
          Material={0.5,0.5,1,0.5});
        WrenchPort InPortRoll annotation (extent=[-40, 80; -20, 100]);
        SI.Force[3] F1;
        SI.Torque[3] M1;
      equation
        Disc.S = T;
        Disc.r = r;
        Rod.S = T;
        Rod.r = r;
        F = m*Gravity + InPortRoll.F + F1;
        M = InPortRoll.M + cross(InPortRoll.P - r, InPortRoll.F) + M1;
        annotation (Diagram);
      end RollingDisc;

      model Disc_on_Base
        extends Roll;
        parameter Real n[3]={0,1,0};
        parameter SI.Length d=0;
        //  parameter SI.Length R = 1;
        SI.Position[3] tau;
        SI.Position[3] ri;
      equation
        n*r = d;
        ri = transpose(InPortB.T)*(r - InPortB.r);
        ri[3] = 0;
        //  ri[1]^2 + ri[2]^2 = R^2;
        tau = {-ri[2],ri[1],0};
        n*(InPortB.T*tau) = 0;

        annotation (Diagram);
      end Disc_on_Base;

      model Test_Disc
        import ThreeD_MBS_Dynamics;
        parameter SI.Acceleration g=9.81;
        parameter SI.Acceleration[3] Gravity={0,-g,0};
        parameter SI.Radius r1=2;
        parameter SI.Mass m_disc=1;
        parameter SI.AngularVelocity omega0=1;
        parameter Real alpha=Modelica.Constants.pi/4;
        Base Base1 annotation (extent=[-80, -20; -40, 20]);
        Disc_on_Base Disc_on_Plane1 annotation (extent=[-20, -20; 20, 20]);
        RollingDisc RollingDisc1(
          Gravity=Gravity,
          r1=r1,
          F1=zeros(3),
          M1=zeros(3),
          q(start={cos(alpha/2),sin(alpha/2),0,0}),
          I=m_disc*[0.25*r1^2, 0, 0; 0, 0.25*r1^2, 0; 0, 0, 0.5*r1^2],
          v(start={-r1*omega0,0,0}),
          omega(start={0,-omega0*sin(alpha),omega0*cos(alpha)}),
          r(start={0,r1*cos(alpha),r1*sin(alpha)})) 
          annotation (extent=[40, -20; 80, 20]);
        annotation (
          Diagram,
          experiment(
            StopTime=50,
            NumberOfIntervals=25000,
            Tolerance=1e-009),
          experimentSetupOutput);
      equation
        connect(Disc_on_Plane1.OutPortB, RollingDisc1.InPortRoll) 
          annotation (points=[6, 18; 6, 30; 54, 30; 54, 18]);
        connect(Base1.OutPort, Disc_on_Plane1.InPortA) 
          annotation (points=[-60,-18; -60,-30; -6,-30; -6,-18]);
        connect(Disc_on_Plane1.InPortB, RollingDisc1.OutPort) 
          annotation (points=[6, -18; 6, -30; 60, -30; 60, -18]);
      end Test_Disc;
    end RollingDisc;

    package Snakeboard

      model Disc
        extends RollingDisc.RollingDisc;
        annotation (Diagram);
        WrenchPort InPortJoint annotation (extent=[20,80; 40,100]);
      equation
        F1 = InPortJoint.F;
        M1 = InPortJoint.M + cross(InPortJoint.P - r, InPortJoint.F);
      end Disc;

      model RodUni
        extends RigidBody;
        parameter SI.Acceleration[3] Gravity;
        parameter SI.Length L;
        parameter SI.Radius r2;
        VisualShape Rod(
          r0={0,0,-L/2},
          Shape="cylinder",
          LengthDirection={0,0,1},
          WidthDirection={1,0,0},
          Length=L,
          Width=2*r2,
          Height=2*r2,
          Material={1,0.5,0.5,0.5});
        WrenchPort InPortJointA annotation (extent=[-10,80; 10,100]);
        annotation (Diagram);
      equation
        Rod.S = T;
        Rod.r = r;
        F = m*Gravity + InPortJointA.F;
        M = InPortJointA.M + cross(InPortJointA.P - r, InPortJointA.F);
      end RodUni;

      model Rod
        extends RigidBodyEnergized;
        parameter SI.Acceleration[3] Gravity;
        parameter SI.Length L;
        parameter SI.Radius r2;
        VisualShape Rod(
          r0={0,0,-L/2},
          Shape="cylinder",
          LengthDirection={0,0,1},
          WidthDirection={1,0,0},
          Length=L,
          Width=2*r2,
          Height=2*r2,
          Material={0.5,0.5,1,0.5});
        WrenchPort InPortJointA annotation (extent=[-40,80; -20,100]);
        WrenchPort InPortJointB annotation (extent=[20,80; 40,100]);
        annotation (Diagram);
      equation
        Rod.S = T;
        Rod.r = r;
        F = m*Gravity + InPortJointA.F + InPortJointB.F;
        M = InPortJointA.M + InPortJointB.M + cross(InPortJointA.P - r,
          InPortJointA.F) + cross(InPortJointB.P - r, InPortJointB.F) + {0,-0.3,0}*
          sin(time);
        //    InPortJointA.F) + cross(InPortJointB.P - r, InPortJointB.F);
      end Rod;

      model Rod_1
        extends RigidBodyEnergized;
        parameter SI.Acceleration[3] Gravity;
        parameter SI.Length L;
        parameter SI.Length Wp;
        parameter SI.Radius r2;
        // Relationship between Vertical and Horizontal Length of Rod_1
        parameter Real coef;
        VisualShape RodH(
          r0={0,0,-L/2},
          Shape="cylinder",
          LengthDirection={0,0,1},
          WidthDirection={1,0,0},
          Length=L,
          Width=2*r2,
          Height=2*r2,
          Material={0.5,0.5,1,0.5});
        VisualShape RodV(
          r0={0,0,0},
          Shape="cylinder",
          LengthDirection={0,1,0},
          WidthDirection={1,0,0},
          Length=L*coef,
          Width=2*r2,
          Height=2*r2,
          Material={0.5,0.5,1,0.5});
        VisualShape Platforma(
          r0={0,L*coef,-L/2 - 0.1},
          Shape="box",
          LengthDirection={0,0,1},
          WidthDirection={1,0,0},
          Length=L + 0.2,
          Width=Wp,
          Height=0.1,
          Material={0.5,0.5,1,0.5});
        WrenchPort InPortJointA annotation (extent=[-60, 80; -40, 100]);
        WrenchPort InPortJointB annotation (extent=[40, 80; 60, 100]);
        annotation (Diagram);
        WrenchPort InPortJoint annotation (extent=[-10, 80; 10, 100]);
      equation
        RodH.S = T;
        RodH.r = r;
        RodV.S = T;
        RodV.r = r;
        Platforma.S = T;
        Platforma.r = r;
        F = m*Gravity + InPortJointA.F + InPortJointB.F + InPortJoint.F;
        M = InPortJointA.M + InPortJointB.M + InPortJoint.M + cross(InPortJointA.P
           - r, InPortJointA.F) + cross(InPortJointB.P - r, InPortJointB.F) + cross(
          InPortJoint.P - r, InPortJoint.F);
      end Rod_1;

      model RollingWheelSet
        parameter SI.Acceleration g = 9.81;
        parameter SI.Acceleration[3] Gravity = {0,-g,0};
        parameter SI.Mass m_rod = 0.5;
        parameter SI.Mass m_disc = 0.5;
        parameter SI.Mass m = Rod1.m + 2*Disc1.m;
        parameter SI.Radius r1 = 0.5;
        parameter SI.Radius r2 = 0.1;
        parameter SI.Length L = 1.5;
        parameter SI.Length Wp = 1.2;
        parameter Real coef = 0.5;
        parameter Real SmallPar = 1; // Parameter of the wheelset simplification
        parameter Stiffness c = 1000;
        parameter Viscosity d = 500;
        parameter SI.AngularVelocity omega0 = 0;
        parameter SI.AngularVelocity omega1 = 0.3;
        parameter Real phi = 0.1;
        parameter SI.Position[3] r0 = {0,r1,0};
        parameter SI.Velocity[3] v0 = {-cos(phi),0,sin(phi)};
        parameter SI.Velocity[3] v1 = v0 + cross({0, omega1, 0}, 0.5*L*{sin(phi), 0, cos(phi)});
        parameter SI.Velocity[3] v2 = v0 + cross({0, omega1, 0}, -0.5*L*{sin(phi), 0, cos(phi)});
        parameter SI.AngularVelocity[3] om01 = {0, omega1, 0} - {0, 0, v1*{cos(phi), 0, -sin(phi)}/r1};
        SpringJoint Joint1(
          c = c,
          d = d,
          nA = {0, 0, 1},
          nB = {0, 0, 1},
          rA = {0, 0, 0},
          rB = {0, 0, 0.5*L}) annotation (extent=[-70,30; -30,70]);
        SpringJoint Joint2(
          c = c,
          d = d,
          nA = {0, 0, 1},
          nB = {0, 0, 1},
          rA = {0, 0, 0},
          rB = {0, 0, -0.5*L}) annotation (extent=[70,30; 30,70]);
        Disc Disc1(
          Gravity = Gravity,
          m = SmallPar*m_disc,
          r1 = r1,
          q(start = {cos(phi/2), 0, sin(phi/2), 0}),
          omega(start = {0, omega1, 0} - {0, 0, v1*{cos(phi),0,-sin(phi)}/r1}),
          v(start = v1),
          I = SmallPar*m_disc*[0.25*r1^2,         0,        0;
                                       0, 0.25*r1^2,        0;
                                       0,         0, 0.5*r1^2],
          r(start = r0 + 0.5*L*{sin(phi), 0, cos(phi)})) 
          annotation (extent=[-70,-20; -30,20], rotation=90);
        Disc Disc2(
          Gravity = Gravity,
          m = SmallPar*m_disc,
          r1 = r1,
          omega(start = {0, omega1, 0} - {0, 0, v2*{cos(phi), 0, -sin(phi)}/r1}),
          v(start = v2),
          q(start = {cos(phi/2), 0, sin(phi/2), 0}),
          I = SmallPar*m_disc*[0.25*r1^2,         0,        0;
                                       0, 0.25*r1^2,        0;
                                       0,         0, 0.5*r1^2],
          r(start = r0 - 0.5*L*{sin(phi), 0, cos(phi)})) 
          annotation (extent=[70,-20; 30,20], rotation=90);
        RollingDisc.Disc_on_Base Disc_on_Plane1(
          ri(start = {0, -r1, 0}),
          r(start = r0 + 0.5*L*{sin(phi), 0, cos(phi)} + {0, -r1, 0})) annotation (extent=[-70,-70; -30,-30], rotation=90);
        RollingDisc.Disc_on_Base Disc_on_Plane2(
          ri(start = {0, -r1, 0}),
          r(start = r0 - 0.5*L*{sin(phi), 0, cos(phi)} + {0, -r1, 0})) annotation (extent=[70,-70; 30,-30], rotation=90);
        Rod_1 Rod1(
          Gravity = Gravity,
          coef = coef,
          m = SmallPar*(m_rod + coef*m_rod),
          r2 = r2,
          L = L,
          Wp = Wp,
          q(start = {cos(phi/2), 0, sin(phi/2), 0}),
          I = m_rod/12*[L^2 + 3*r2^2 + 4*L^2*coef^3 + 3*coef*r2^2,                          0,                                   0;
                                                                0, L^2 + 3*r2^2 + 6*coef*r2^2,                                   0;
                                                                0,                          0, 6*r2^2 + 4*L^2*coef^3 + 3*coef*r2^2],
          v(start=v0),
          omega(start={0,omega1,omega0}),
          r(start=r0)) annotation (extent=[-20,30; 20,70]);
        annotation (
          Diagram,
          experiment(
            StopTime=10,
            NumberOfIntervals=5000,
            Tolerance=1e-012),
          experimentSetupOutput);
        WrenchPort InPortJoint annotation (extent=[-10, 80; 10, 100]);
        KinematicPort OutPortJoint annotation (extent=[20,-100; 40,-80]);
        KinematicPort OutPortBase annotation (extent=[-40,-100; -20,-80]);
        SI.Energy Ed;
      equation
        Ed = Joint1.Ed + Joint2.Ed;
        connect(Joint1.OutPortB, Rod1.InPortJointA) 
          annotation (points=[-44,68; -44,76; -10,76; -10,68]);
        connect(Rod1.InPortJointB, Joint2.OutPortB) 
          annotation (points=[10,68; 10,76; 44,76; 44,68]);
        connect(Joint1.InPortB, Rod1.OutPort) 
          annotation (points=[-44,32; -44,26; 0,26; 0,32]);
        connect(Joint2.InPortB, Rod1.OutPort) 
          annotation (points=[44,32; 44,26; 0,26; 0,32]);
        connect(Rod1.InPortJoint, InPortJoint) annotation (points=[0,68; 0,90]);
        connect(Rod1.OutPort, OutPortJoint) 
          annotation (points=[0,32; 0,-76; 30,-76; 30,-90]);
        connect(Disc1.InPortJoint, Joint1.OutPortA) annotation (points=[-68,6;
              -76,6; -76,76; -56,76; -56,68], style(color=3, rgbcolor={0,0,255}));
        connect(Disc1.InPortRoll, Disc_on_Plane1.OutPortB) annotation (points=[-68,-6;
              -76,-6; -76,-44; -68,-44],         style(color=3, rgbcolor={0,0,
                255}));
        connect(Joint2.OutPortA, Disc2.InPortJoint) annotation (points=[56,68;
              56,76; 76,76; 76,6; 68,6], style(color=3, rgbcolor={0,0,255}));
        connect(Disc2.InPortRoll, Disc_on_Plane2.OutPortB) annotation (points=[68,-6;
              76,-6; 76,-44; 68,-44],        style(color=3, rgbcolor={0,0,255}));
        connect(Joint2.InPortA, Disc2.OutPort) annotation (points=[56,32; 56,22;
              20,22; 20,-1.10215e-015; 32,-1.10215e-015], style(color=3,
              rgbcolor={0,0,255}));
        connect(Joint1.InPortA, Disc1.OutPort) annotation (points=[-56,32; -56,
              22; -20,22; -20,-1.10215e-015; -32,-1.10215e-015], style(color=3,
              rgbcolor={0,0,255}));
        connect(Disc_on_Plane1.InPortA, OutPortBase) annotation (points=[-32,
              -56; -10,-56; -10,-90; -30,-90], style(color=3, rgbcolor={0,0,255}));
        connect(Disc_on_Plane2.InPortA, OutPortBase) annotation (points=[32,-56;
              10,-56; 10,-90; -30,-90], style(color=3, rgbcolor={0,0,255}));
        connect(Disc1.OutPort, Disc_on_Plane1.InPortB) annotation (points=[-32,
              -1.10215e-015; -26,-1.10215e-015; -26,0; -20,0; -20,-44; -32,-44],
            style(color=3, rgbcolor={0,0,255}));
        connect(Disc2.OutPort, Disc_on_Plane2.InPortB) annotation (points=[32,
              -1.10215e-015; 26,-1.10215e-015; 26,0; 20,0; 20,-44; 32,-44], style(
              color=3, rgbcolor={0,0,255}));
      end RollingWheelSet;

      model SemiBar
        extends RigidBodyEnergized;
        parameter SI.Acceleration[3] Gravity;
        parameter SI.Length LD;
        parameter SI.Length WD;
        parameter SI.Radius HD;
        VisualShape Bar(
          r0={-LD/2,0,0},
          Shape="box",
          LengthDirection={1,0,0},
          WidthDirection={0,0,1},
          Length=LD,
          Width=WD,
          Height=HD,
          Material={0,0.7,0.7,0.5});
        WrenchPort InPortJointA annotation (extent=[-60, 80; -40, 100]);
        annotation (Diagram);
      equation
        Bar.S = T;
        Bar.r = r;
        F = m*Gravity + InPortJointA.F;
        M = InPortJointA.M + cross(InPortJointA.P - r, InPortJointA.F);
      end SemiBar;

      model LeftBar
        extends RigidBodyEnergized;
        parameter SI.Acceleration[3] Gravity;
        parameter SI.Length LD;
        parameter SI.Length WD;
        parameter SI.Radius HD;
        VisualShape Bar(
          r0={-LD/4,0,0},
          Shape="box",
          LengthDirection={1,0,0},
          WidthDirection={0,0,1},
          Length=LD,
          Width=WD,
          Height=HD,
          Material={0,0.7,0.7,0.5});
        WrenchPort InPortJointA annotation (extent=[-60, 80; -40, 100]);
        WrenchPort InPortJointB annotation (extent=[40, 80; 60, 100]);
        WrenchPort InPortJoint annotation (extent=[-10, 80; 10, 100]);
        annotation (Diagram);
      equation
        Bar.S = T;
        Bar.r = r;
      //  F = m*Gravity + InPortJointA.F + InPortJointB.F + InPortJoint.F + T*{1,0,0};
        F = m*Gravity + InPortJointA.F + InPortJointB.F + InPortJoint.F;
        M = InPortJointA.M + InPortJointB.M + InPortJoint.M + cross(InPortJointA.P
           - r, InPortJointA.F) + cross(InPortJointB.P - r, InPortJointB.F) + cross(
          InPortJoint.P - r, InPortJoint.F);
      end LeftBar;

      model RightBar
        extends RigidBodyEnergized;
        parameter SI.Acceleration[3] Gravity;
        WrenchPort InPortJointA annotation (extent=[-50, 80; -30, 100]);
        WrenchPort InPortJointB annotation (extent=[30, 80; 50, 100]);
        annotation (Diagram);
      equation
        F = m*Gravity + InPortJointA.F + InPortJointB.F;
      //  F = m*Gravity + InPortJointA.F + InPortJointB.F;
        M = InPortJointA.M + InPortJointB.M + cross(InPortJointA.P - r, InPortJointA.F) + cross(InPortJointB.P - r, InPortJointB.F);
      end RightBar;

      model Rotor
        extends RigidBodyEnergized;
        parameter SI.Acceleration[3] Gravity;
        // Disc radius
        parameter SI.Radius r3;
        parameter SI.Length HR;
        VisualShape Rotor(
          r0={0,-HR/2,0},
          Shape="cylinder",
          LengthDirection={0,1,0},
          WidthDirection={1,0,0},
          Length=HR,
          Width=2*r3,
          Height=2*r3,
          Material={0.5,0.8,1,0.2});
        VisualShape Rod(
          r0={0,0,0.02},
          Shape="box",
          LengthDirection={1,0,0},
          WidthDirection={0,0,1},
          Length=r3 - 0.05,
          Width=0.02,
          Height=HR + 0.05,
          Material={0.5,0.5,1,0.5});
        WrenchPort InPort annotation (extent=[-10, 80; 10, 100]);
      equation
        Rotor.S = T;
        Rotor.r = r;
        Rod.S = T;
        Rod.r = r;
        F = m*Gravity + InPort.F;
        M = InPort.M + cross(InPort.P - r, InPort.F);
        annotation (Diagram);
      end Rotor;

      model Test_HalfWheelSet
        parameter SI.Acceleration g=9.81;
        parameter SI.Acceleration[3] Gravity={0,-g,0};
        parameter SI.Mass m_rod=0.5;
        parameter SI.Mass m_disc=1;
        parameter SI.Radius r1=2;
        parameter SI.Radius r2=0.1;
        parameter SI.Length L=2;
        parameter SI.AngularVelocity omega0=1;
        parameter Real alpha=Modelica.Constants.pi;
        Disc Disc1(
          Gravity=Gravity,
          r1=r1,
          q(start={1,0,0,0}),
          I=m_disc*[0.25*r1^2, 0, 0; 0, 0.25*r1^2, 0; 0, 0, 0.5*r1^2],
          v(start={-r1*omega0,0,0}),
          omega(start={0,0,omega0}),
          r(start={0,r1,L/2})) annotation (extent=[-44,10; -4,50]);
        Disc_on_Base Disc_on_Plane1 annotation (extent=[-92,10; -52,50]);
        Base Base1 annotation (extent=[-20,-50; 20,-10]);
        annotation (
          Diagram,
          experiment(
            StopTime=10,
            NumberOfIntervals=5000,
            Tolerance=1e-009),
          experimentSetupOutput);
        RodUni Rod1(
          Gravity=Gravity,
          r2=r2,
          m=m_rod,
          L=L,
          q(start={1,0,0,0}),
          I=0.25*m_rod*[L^2/3 + r2^2, 0, 0; 0, L^2/3 + r2^2, 0; 0, 0, 2*r2^2],
          v(start={-r1*omega0,0,0}),
          omega(start={0,0,0}),
          r(start={0,r1,0})) annotation (extent=[52,10; 92,50]);
        FixedJoint Joint1(
          nA={0,0,1},
          nB={0,0,1},
          rA={0,0,0},
          rB={0,0,0.5*L}) annotation (extent=[4,10; 44,50]);
      //  FreeJoint Joint1(
      //    bA={1,0,0},
      equation
        connect(Disc_on_Plane1.InPortB, Disc1.OutPort) 
          annotation (points=[-66,12; -66,0; -24,0; -24,12]);
        connect(Joint1.InPortB, Rod1.OutPort) 
          annotation (points=[30,12; 30,0; 72,0; 72,12]);
        connect(Disc1.InPortJoint, Joint1.OutPortA) 
          annotation (points=[-18,48; -18,60; 18,60; 18,48]);
        connect(Joint1.InPortA, Disc1.OutPort) 
          annotation (points=[18,12; 18,0; -24,0; -24,12]);
        connect(Disc1.InPortRoll, Disc_on_Plane1.OutPortB) annotation (points=[
              -30,48; -30,60; -66,60; -66,48], style(color=3, rgbcolor={0,0,255}));
        connect(Joint1.OutPortB, Rod1.InPortJointA) annotation (points=[30,48;
              30,60; 72,60; 72,48], style(color=3, rgbcolor={0,0,255}));
        connect(Disc_on_Plane1.InPortA, Base1.OutPort) annotation (points=[-78,
              12; -78,-60; 0,-60; 0,-48], style(color=3, rgbcolor={0,0,255}));
      end Test_HalfWheelSet;

      model Test_DAE
        annotation (
          Diagram,
          experiment(
            StopTime=50,
            Interval=30000,
            Tolerance=1e-012),
          experimentSetupOutput,
          DymolaStoredErrors);
        parameter SI.Acceleration g=9.81;
        parameter SI.Length r1=0.5;
        parameter SI.Length r2=0.1;
        parameter SI.Length L=1.5;
        parameter SI.Mass md=0.5;
        parameter SI.Mass mr=0.5;
        parameter SI.Mass M=mr + 2*md;
        parameter SI.MomentOfInertia Idy=md*0.25*r1^2;
        parameter SI.MomentOfInertia Idz=md*0.5*r1^2;
        parameter SI.MomentOfInertia Irz=mr*0.5*r2^2;
        parameter SI.MomentOfInertia Iy=0.25*mr*(L^2/3 + r2^2) + 0.5*md*L^2 + 2*Idy;
        parameter Real phi0=0.1;
        parameter SI.AngularVelocity om0=0.3;
        parameter SI.Velocity[3] v0={-cos(phi0),0,sin(phi0)};
        parameter SI.Velocity[3] v10=v0 + cross({0,om0,0}, 0.5*L*{sin(phi0),0,cos(
            phi0)});
        parameter SI.Velocity[3] v20=v0 + cross({0,om0,0}, -0.5*L*{sin(phi0),0,cos(
            phi0)});
        parameter SI.AngularVelocity om10=-sqrt(v10*v10)/r1;
        parameter SI.AngularVelocity om20=-sqrt(v20*v20)/r1;
        parameter SI.AngularVelocity omz0=0;
        SI.Angle phi(start=phi0);
        SI.Angle phi1;
        SI.Angle phi2;
        SI.Force Fx1;
        SI.Force Fx2;
        SI.Force Fz;
        //  SI.Force Fz2;
        SI.Force N1;
        SI.Force N2;
        SI.AngularVelocity om(start=om0);
        SI.AngularVelocity om1(start=om10);
        SI.AngularVelocity om2(start=om20);
        SI.AngularVelocity omz(start=omz0);
        SI.AngularAcceleration eps;
        SI.AngularAcceleration eps1;
        SI.AngularAcceleration eps2;
        SI.Velocity v[3];
        SI.Velocity v1[3];
        SI.Velocity v2[3];
        SI.Position r[3](start={0,r1,0});
        SI.Position ksi;
        SI.Force B;
        SI.Torque M1;
      equation
        der(r) = v;
        der(phi) = om;
        der(phi1) = om1;
        der(phi2) = om2;
        der(om) = eps;
        der(om1) = eps1;
        der(om2) = eps2;
        der(omz) = 0;
        M*r1^2*0.5/L*(om2^2 - om1^2) = Fz;
        -0.5*M*r1*(eps1 + eps2) = Fx1 + Fx2;
        Idz*eps1 = r1*Fx1;
        Idz*eps2 = r1*Fx2;
        Iy*eps = 0.5*L*(Fx1 - Fx2) - 0.3*sin(time);
        Irz*om*omz = 0.5*L*(N2 - N1) + M1;
        B = md*v1*v1/ksi + md*v2*v2/(ksi + L) + mr*v*v/(ksi + 0.5*L);
        Idz*(om1 + om2)*om = -r1*B - M1;
        0 = N1 + N2 - mr*g;
        ksi = om1*L/(om2 - om1);
        om*L = r1*(om2 - om1);
        v = -0.5*r1*(om1 + om2)*{cos(phi),0,-sin(phi)};
        v1 = (-0.5*r1*(om1 + om2) + 0.5*L*om)*{cos(phi),0,-sin(phi)};
        v2 = (-0.5*r1*(om1 + om2) - 0.5*L*om)*{cos(phi),0,-sin(phi)};

      end Test_DAE;

      model Test_RollWheelSet
        parameter SI.Acceleration g=9.81;
        parameter SI.Acceleration[3] Gravity={0,-g,0};
        parameter SI.Mass m_rod=0.5;
        parameter SI.Mass m_disc=0.5;
        parameter SI.Radius r1=0.5;
        parameter SI.Radius r2=0.1;
        parameter SI.Length L=1.5;
        parameter Real coef=0.5;
        parameter SI.AngularVelocity omegaz=0;
        parameter SI.AngularVelocity omega1=0.3;
        parameter SI.Angle phi=0.1;
        parameter SI.Position[3] r0={0,r1,0};
        parameter SI.Velocity[3] v0={-cos(phi),0,sin(phi)};
        parameter SI.Velocity[3] v1=v0 + cross({0,omega1,0}, 0.5*L*{sin(phi),0,cos(phi)});
        parameter SI.Velocity[3] v2=v0 + cross({0,omega1,0}, -0.5*L*{sin(phi),0,cos(phi)});
        parameter SI.Velocity[3] om01={0,omega1,0} - {0,0,v1*{cos(phi),0,-sin(phi)}/r1};
      //  SI.Energy Ed;
        SI.Energy E;
        SI.Angle d_phi;
        SI.Position d_x;
        SI.Position d_z;
        SI.Angle phi_pr(start=phi);
        SI.Angle angle;
        SI.Force N;
        Test_DAE Test_DAE1(
          phi0=phi,
          om0=omega1,
          om10=-v1*{cos(phi),0,-sin(phi)}/r1,
          om20=-v2*{cos(phi),0,-sin(phi)}/r1,
          omz0=omegaz,
          v0=v0,
          v10=v1,
          v20=v2,
          r1=r1,
          r2=r2,
          L=L,
          md=m_disc,
          mr=m_rod) annotation (extent=[-20,-26; 20,14]);
        SpringJoint Joint1(
          nA={0,0,1},
          nB={0,0,1},
          rA={0,0,0},
          rB={0,0,0.5*L}) annotation (extent=[-80,34; -40,74]);
      //    c = 10,
      //    d = 10,
        SpringJoint Joint2(
          nA={0,0,1},
          nB={0,0,1},
          rA={0,0,0},
          rB={0,0,-0.5*L}) annotation (extent=[80,34; 40,74]);
      //    c = 10,
      //    d = 10,
        Disc Disc1(
          Gravity=Gravity,
          m=m_disc,
          r1=r1,
          q(start={cos(phi/2),0,sin(phi/2),0}),
          omega(start={0,omega1,0} - {0,0,v1*{cos(phi),0,-sin(phi)}/r1}),
          v(start=v1),
          I=m_disc*[0.25*r1^2, 0, 0; 0, 0.25*r1^2, 0; 0, 0, 0.5*r1^2],
          r(start=r0 + 0.5*L*{sin(phi),0,cos(phi)})) 
          annotation (extent=[-80,-26; -40,14], rotation=90);
        Disc Disc2(
          Gravity=Gravity,
          m=m_disc,
          r1=r1,
          omega(start={0,omega1,0} - {0,0,v2*{cos(phi),0,-sin(phi)}/r1}),
          v(start=v2),
          q(start={cos(phi/2),0,sin(phi/2),0}),
          I=m_disc*[0.25*r1^2, 0, 0; 0, 0.25*r1^2, 0; 0, 0, 0.5*r1^2],
          r(start=r0 - 0.5*L*{sin(phi),0,cos(phi)})) 
          annotation (extent=[40,14; 80,-26], rotation=-90);
        Disc_on_Base Disc_on_Plane1(
          ri(start={0,-r1,0}),
          r(start=r0 + 0.5*L*{sin(phi),0,cos(phi)} + {0,-r1,0})) annotation (extent=[-80,-76; -40,-36], rotation=90);
      //    R=r1,
        Disc_on_Base Disc_on_Plane2(
          ri(start={0,-r1,0}),
          r(start=r0 - 0.5*L*{sin(phi),0,cos(phi)} + {0,-r1,0})) annotation (extent=[40,-36; 80,-76], rotation=-90);
      //    R=r1,
        Rod Rod1(
          Gravity=Gravity,
          m=m_rod,
          r2=r2,
          L=L,
          q(start={cos(phi/2),0,sin(phi/2),0}),
          I=0.25*m_rod*[L^2/3 + r2^2, 0, 0; 0, L^2/3 + r2^2, 0; 0, 0, 2*r2^2],
          v(start=v0),
          omega(start={0,omega1,omegaz}),
          r(start=r0)) annotation (extent=[-20,34; 20,74]);
        annotation (
          Diagram,
          experiment(
            StopTime=50,
            NumberOfIntervals=30000,
            Tolerance=1e-012),
          experimentSetupOutput);
        Base Base1 annotation (extent=[-20,-76; 20,-36]);
        Real[3] i;
        Real[3] j;
        Real[3] k;
        SI.Torque M1;
        SI.Torque M2;
        SI.Torque M3;
        SI.Torque M4;
      equation
        i = cross(j, k);
        j = {0,1,0};
        k = Rod1.T*{0,0,1};
        M1 = Rod1.InPortJointA.M*k;
        M2 = Rod1.InPortJointB.M*k;
        M3 = cross(Rod1.InPortJointA.P - Rod1.r, Rod1.InPortJointA.F)*k;
        M4 = cross(Rod1.InPortJointB.P - Rod1.r, Rod1.InPortJointB.F)*k;
      //  Ed = Joint1.Ed + Joint2.Ed;
        E = Disc1.K + Disc2.K + Rod1.K;
        angle = Modelica.Math.asin(Rod1.T*{0,0,1}*{0,1,0});
        d_phi = -Test_DAE1.phi + phi_pr;
        d_x = -Test_DAE1.r[1] + Rod1.r[1];
        d_z = -Test_DAE1.r[3] + Rod1.r[3];
        der(phi_pr) = Rod1.OutPort.omega[2];
        N = Disc_on_Plane2.OutPortB.F*Disc_on_Plane2.n - Disc_on_Plane1.OutPortB.F*
          Disc_on_Plane1.n;
        connect(Joint1.OutPortB, Rod1.InPortJointA) 
          annotation (points=[-54,72; -54,84; -6,84; -6,72]);
        connect(Rod1.InPortJointB, Joint2.OutPortB) 
          annotation (points=[6,72; 6,84; 54,84; 54,72]);
        connect(Joint1.InPortB, Rod1.OutPort) 
          annotation (points=[-54,36; -54,26; 0,26; 0,36]);
        connect(Joint2.InPortB, Rod1.OutPort) 
          annotation (points=[54,36; 54,26; 0,26; 0,36]);
        connect(Disc_on_Plane1.InPortA, Base1.OutPort) annotation (points=[-42,-62;
              -30,-62; -30,-86; 0,-86; 0,-74], style(color=3, rgbcolor={0,0,255}));
        connect(Disc_on_Plane2.InPortA, Base1.OutPort) annotation (points=[42,-62; 30,
              -62; 30,-86; 0,-86; 0,-74], style(color=3, rgbcolor={0,0,255}));
        connect(Disc1.OutPort, Disc_on_Plane1.InPortB) annotation (points=[-42,
              -6; -30,-6; -30,-50; -42,-50],
                                         style(color=3, rgbcolor={0,0,255}));
        connect(Disc2.OutPort, Disc_on_Plane2.InPortB) annotation (points=[42,
              -6; 30,-6; 30,-50; 42,-50],
                                   style(color=3, rgbcolor={0,0,255}));
        connect(Disc2.InPortRoll, Disc_on_Plane2.OutPortB) annotation (points=[78,-12;
              90,-12; 90,-50; 78,-50], style(color=3, rgbcolor={0,0,255}));
        connect(Disc1.InPortRoll, Disc_on_Plane1.OutPortB) annotation (points=[-78,
              -12; -90,-12; -90,-50; -78,-50], style(color=3, rgbcolor={0,0,255}));
        connect(Joint2.InPortA, Disc2.OutPort) annotation (points=[66,36; 66,22;
              30,22; 30,-6; 42,-6],
                                 style(color=3, rgbcolor={0,0,255}));
        connect(Joint1.InPortA, Disc1.OutPort) annotation (points=[-66,36; -66,
              22; -30,22; -30,-6; -42,-6],
                                       style(color=3, rgbcolor={0,0,255}));
        connect(Joint2.OutPortA, Disc2.InPortJoint) annotation (points=[66,72;
              66,84; 90,84; 90,-8.88178e-016; 78,-8.88178e-016],
                                                          style(color=3, rgbcolor={0,
                0,255}));
        connect(Joint1.OutPortA, Disc1.InPortJoint) annotation (points=[-66,72;
              -66,84; -90,84; -90,8.88178e-016; -78,8.88178e-016],
                                                               style(color=3,
              rgbcolor={0,0,255}));
      end Test_RollWheelSet;

      model Test_WheelSet_with_Bar
        // One WheelSet with Bar
        parameter SI.Acceleration g=9.81;
        parameter SI.Acceleration[3] Gravity={0,-g,0};
        parameter SI.Mass m_bar=1;
        // Radius of Disc
        parameter SI.Radius r1=1;
        // Radius of Rod
        parameter SI.Radius r2=0.1;
        // Radius of Initial Position
        parameter SI.Radius R=1;
        // Horizontal Length of Rod
        parameter SI.Length L=3;
        // Length of Bar
        parameter SI.Length LD=8;
        // Width of Bar
        parameter SI.Length WD=0.8;
        // Height of Bar
        parameter SI.Length HD=0.2;
        parameter SI.Length h=0.1 + r2 + HD/2;
        // parameter SI.AngularVelocity omega0=1;
        parameter SI.AngularVelocity omega1=0.8;
        parameter SI.Angle phi1=0.1;
        //  parameter Real phi1=Modelica.Constants.pi/6;
        parameter SI.Angle phi2=-Modelica.Constants.pi/8;
        parameter SI.Position[3] r0={-0.5*LD,r1,0};
        //  parameter Real[3] v0={-r1*omega0 - 0.5*L*omega1,0,0};
        parameter SI.Velocity[3] v0={0,0,0};
        RollingWheelSet WheelSet1(
          Gravity=Gravity,
          r1=r1,
          r2=r2,
          L=L,
          r0=r0,
          v0=v0,
          omega1=omega1,
          phi=phi1) annotation (extent=[-44,-20; -4,20]);
        SemiBar Bar1(
          Gravity=Gravity,
          m=m_bar,
          LD=LD,
          WD=WD,
          HD=HD,
          q(start={1,0,0,0}),
          I=m_bar/12*[LD^2 + HD^2, 0, 0; 0, WD^2 + HD^2, 0; 0, 0, LD^2 + WD^2],
          v(start=v0),
          omega(start={0,0,0}),
          r(start={0,r1 + h,0})) annotation (extent=[52,-20; 92,20]);
        Base Base1 annotation (extent=[-92,-20; -52,20]);
        FixedJoint Joint1(
          nA={0,1,0},
          nB={0,1,0},
          rA={0,h,0},
          rB={-0.5*LD,0,0}) annotation (extent=[4,-20; 44,20]);
        annotation (
          Diagram,
          experiment(
            StopTime=30,
            NumberOfIntervals=10000,
            Tolerance=1e-012),
          experimentSetupOutput,
          DymolaStoredErrors);
      equation
        connect(WheelSet1.OutPortBase, Base1.OutPort) 
          annotation (points=[-30,-18; -30,-30; -72,-30; -72,-18]);
        connect(WheelSet1.InPortJoint, Joint1.OutPortA) 
          annotation (points=[-24,18; -24,30; 18,30; 18,18]);
        connect(Joint1.OutPortB, Bar1.InPortJointA) 
          annotation (points=[30,18; 30,30; 62,30; 62,18]);
        connect(WheelSet1.OutPortJoint, Joint1.InPortA) 
          annotation (points=[-18,-18; -18,-30; 18,-30; 18,-18]);
        connect(Joint1.InPortB, Bar1.OutPort) 
          annotation (points=[30,-18; 30,-30; 72,-30; 72,-18]);
      end Test_WheelSet_with_Bar;

      model Test_Kuleshov
        parameter SI.Mass m=5;
        parameter SI.Length L=2.5;
        parameter SI.MomentOfInertia J=0.1442;
        parameter SI.MomentOfInertia Jr=0.125;
        parameter SI.MomentOfInertia Jp=0.1275;
        parameter SI.Angle a_psi=0.7;
        parameter SI.Angle b_psi=0;
        parameter SI.AngularVelocity w0_psi=1;
        parameter SI.Angle a_phi=0.3;
        parameter SI.Angle b_phi=0.01;
        parameter SI.AngularVelocity w0_phi=1;
        SI.Angle psi;
        SI.Angle phi;
        SI.Angle theta;
        SI.AngularVelocity w_theta;
        SI.AngularVelocity w_phi;
        SI.AngularVelocity w_psi;
        SI.Torque u_psi;
        SI.Torque u_f;
        SI.Torque u_b;
        SI.Position x;
        SI.Position y;
        SI.Position z;
        SI.Velocity vx;
        SI.Velocity vy;
        SI.Velocity vz;
        SI.Velocity v;
        SI.Acceleration ax;
        SI.Acceleration ay;
        SI.Energy K;
      equation
        z = -y;
        der(psi) = w_psi;
        der(phi) = w_phi;
        der(theta) = w_theta;
        der(x) = vx;
        vx = L*der(theta)*cos(theta)/tan(phi);
        der(y) = vy;
        vy = L*der(theta)*sin(theta)/tan(phi);
        vz = -vy;
        der(vx) = ax;
        der(vy) = ay;
        der(w_theta) - (m*L^2)*der(phi)/tan(phi)*w_theta/((J + Jr + 2*Jp)*(sin(phi))
          ^2 + m*L^2*(cos(phi))^2) + Jr*der(w_psi)*(sin(phi))^2/((J + Jr + 2*Jp)*(
          sin(phi))^2 + m*L^2*(cos(phi))^2) = 0;
        Jr*(der(w_psi) + der(w_theta)) = u_psi;
        Jp*(der(w_phi) + der(w_theta)) = u_f;
        Jp*(der(w_theta) - der(w_phi)) = u_b;
        psi = a_psi*sin(w0_psi*time + b_psi);
        phi = a_phi*sin(w0_phi*time + b_phi);
        v = sqrt(vx^2 + vy^2);
        K = 0.5*m*v*v + 0.5*Jp*(w_theta + w_phi)^2 + 0.5*Jp*(w_theta - w_phi)^2 +
          0.5*J*(w_theta)^2 + 0.5*Jr*(w_psi + w_theta)^2;
        annotation (
          Diagram,
          experiment(
            StopTime=25,
            NumberOfIntervals=8000,
            Tolerance=1e-012),
          experimentSetupOutput,
          DymolaStoredErrors);
      end Test_Kuleshov;

      model TestSnakeboardSectioned
        // With Rotor and Control Phi & Psi
        parameter SI.Acceleration g = 9.81;
        parameter SI.Acceleration[3] Gravity = {0,-g,0};
        parameter SI.Velocity delta = 10^(-4);
        parameter SI.Mass m_bar = 1;
        parameter SI.Mass m_rot = 1;
        parameter SI.Mass m = LeftWheels.m + RightWheels.m + m_bar + m_rot;
        // Radius of Disc
        parameter SI.Radius r1 = 0.4;
        // Radius of Rod
        parameter SI.Radius r2 = 0.15;
        // Horizontal Length of Rod
        // Radius of Rotor
        parameter SI.Radius r3 = 0.5;
        parameter SI.Length L = 1.5;
        // Length of Bar
        parameter SI.Length LD = 5;
        // Width of Bar
        parameter SI.Length WD = 0.5;
        // Height of Bar
        parameter SI.Length HD = 0.2;
        // Height of Rotor
        parameter SI.Length HR = 0.4;
        parameter SI.Length h = 0.2 + r2 + HD/2;
        parameter SI.Length hr = r1 + h + HD/2;
        parameter SI.AngularVelocity omega0 = 0;
        parameter SI.AngularVelocity omega1 = 0.3;
        parameter SI.AngularVelocity omega_r = 1;
        //  parameter Real phi2=-Modelica.Constants.pi/8;
        parameter SI.Position[3] r0 = {-0.5*LD + r2,r1,0};
        parameter SI.Velocity[3] v0 = {0,0,0};
        parameter SI.Angle a_psi = 0.7;
        parameter SI.Angle b_psi = 0;
        parameter SI.Angle a_phi = 0.3;
        parameter SI.Angle b_phi = 0.01;
        parameter SI.Angle phi1 = a_phi*sin(b_phi);
        //  parameter Real phi1=0;
        //  parameter SI.AngularVelocity w_phi=1;
        parameter Stiffness c = 1000;
        parameter Viscosity d = 5000;
        RollingWheelSet LeftWheels(
          Gravity = Gravity,
          c = c,
          d = d,
          r1 = r1,
          r2 = r2,
          L=L,
          r0=r0,
          v0=v0 + cross({0, omega0, 0}, {-0.5*LD + r2, 0, 0}),
          omega1 = omega0 - omega1,
          phi = -phi1) annotation (extent=[-70,-94; -30,-54], rotation=90);
        RollingWheelSet RightWheels(
          Gravity=Gravity,
          c=c,
          d=d,
          r1=r1,
          r2=r2,
          L=L,
          r0=r0 + {LD - 2*r2,0,0},
          v0=v0 + cross({0,omega0,0}, {0.5*LD - r2,0,0}),
          omega1=omega0 + omega1,
          phi=phi1) annotation (extent=[70,-94; 30,-54], rotation=90);
        LeftBar LBar(
          Gravity=Gravity,
          LD=LD,
          WD=WD,
          HD=HD,
          q(start={1,0,0,0}),
          omega(start={0,omega0,0}),
          m=0.5*m_bar,
          I=0.5*m_bar/12*[0.25*LD^2 + HD^2, 0, 0; 0, WD^2 + HD^2, 0; 0, 0, 0.25*LD^
              2 + WD^2],
          r(start={-0.25*LD,r1 + h,0}),
          v(start=v0 + cross({0,omega0,0}, {-LD/4,0,0}))) 
          annotation (extent=[-70,0; -30,40]);
        Base FLoor annotation (extent=[-20,-74; 20,-34]);
        FixedServoJoint LeftJoint(
          nA={0,1,0},
          nB={0,1,0},
          rA={0,h,0},
          rB={-0.25*LD + r2,0,0}) annotation (extent=[-70,-48; -30,-8],  rotation=90);
        FixedServoJoint RightJoint(
          nA={0,1,0},
          nB={0,1,0},
          rA={0,h,0},
          rB={0.25*LD - r2,0,0}) annotation (extent=[70,-48; 30,-8],  rotation=90);
        FixedServoJoint CJoint(
          nA={0,1,0},
          nB={0,1,0},
          rB={0,0,0},
          rA={LD/4,HD/2,0}) annotation (extent=[-20,50; 20,90]);
        Rotor RoBody(
          Gravity=Gravity,
          m=m_rot,
          r3=r3,
          HR=HR,
          q(start={1,0,0,0}),
          I=m_rot*[0.25*r3^2, 0, 0; 0, 0.5*r3^2, 0; 0, 0, 0.25*r3^2],
          v(start=v0),
          omega(start={0,omega_r*a_psi,0}),
          r(start={0,hr,0})) annotation (extent=[30,50; 70,90]);
        SpringJoint Spring(
          c=c,
          d=d,
          nA={1,0,0},
          nB={1,0,0},
          rA={LD/4,0,0},
          rB={-LD/4,0,0}) annotation (extent=[-20,0; 20,40]);
        RightBar RBar(
          Gravity=Gravity,
          q(start={1,0,0,0}),
          m=0.5*m_bar,
          I=0.5*m_bar/12*[0.25*LD^2 + HD^2, 0, 0; 0, WD^2 + HD^2, 0; 0, 0, 0.25*LD^
              2 + WD^2],
          r(start={0.25*LD,r1 + h,0}),
          v(start=v0 + cross({0,omega0,0}, {LD/4,0,0})),
          omega(start={0,omega0,0})) annotation (extent=[30,0; 70,40]);
        annotation (
          Diagram,
          experiment(
            StopTime=50,
            NumberOfIntervals=15000,
            Tolerance=1e-012),
          experimentSetupOutput,
          DymolaStoredErrors);
        Test_Kuleshov IdealModel(
          J=LBar.I[2, 2],
          Jr=RoBody.I[2, 2],
          L=LD/2,
          m=m_bar + m_rot + 2*(2*LeftWheels.m_disc + LeftWheels.m_rod),
          Jp=LeftWheels.Rod1.I[2, 2] + 2*(LeftWheels.Disc1.I[2, 2] + LeftWheels.m_disc*L
              ^2/4),
          a_psi=a_psi,
          b_psi=b_psi,
          w0_psi=omega_r,
          a_phi=a_phi,
          b_phi=b_phi,
          w0_phi=omega1/a_phi/cos(b_phi)) annotation (extent=[-96,50; -56,90]);
        SI.Angle theta;
        SI.Angle psi;
        SI.Angle phi_pr;
        SI.AngularAcceleration eps_psi;
        SI.AngularVelocity om_psi;
        SI.AngularAcceleration eps_phi;
        SI.AngularVelocity om_phi;
        SI.Position delta_x;
        SI.Position delta_z;
        SI.Angle delta_psi;
        SI.Angle delta_phi;
        SI.Torque delta_mom;
        SI.Energy Ed;
        SI.Energy Kt;
        SI.Energy Kr;
      equation
        Ed = LeftWheels.Ed + RightWheels.Ed + Spring.Ed;
        //  K = 0.5*IdealModel.Jp*LeftWheels.Rod1.omega*LeftWheels.Rod1.omega +
        //      0.5*IdealModel.Jp*RightWheels.Rod1.omega*RightWheels.Rod1.omega;
        //  Kt = 0.5*(LeftWheels.Disc1.m + LeftWheels.Disc2.m + LeftWheels.Rod1.m +
        //      RightWheels.Disc1.m + RightWheels.Disc2.m + RightWheels.Rod1.m +
        //      LBar.m + RBar.m + RoBody.m)*RoBody.v*RoBody.v;
        Kt = LBar.K + RBar.K + RoBody.K + 0.5*IdealModel.Jp*LeftWheels.Rod1.
          omega*LeftWheels.Rod1.omega + 0.5*IdealModel.Jp*RightWheels.Rod1.omega*
          RightWheels.Rod1.omega;
        Kr = 0.5*LeftWheels.Disc1.I[3, 3]*LeftWheels.Disc1.omega[3]*LeftWheels.Disc1.
          omega[3] + 0.5*LeftWheels.Disc2.I[3, 3]*LeftWheels.Disc2.omega[3]*LeftWheels.
           Disc2.omega[3] + 0.5*RightWheels.Disc1.I[3, 3]*RightWheels.Disc1.omega[3]*
          RightWheels.Disc1.omega[3] + 0.5*RightWheels.Disc2.I[3, 3]*RightWheels.Disc2.
          omega[3]*RightWheels.Disc2.omega[3] + 0.5*(LeftWheels.Rod1.m + LeftWheels.
          Disc1.m + LeftWheels.Disc2.m)*LeftWheels.Rod1.v*LeftWheels.Rod1.v + 0.5*(
          RightWheels.Rod1.m + RightWheels.Disc1.m + RightWheels.Disc2.m)*RightWheels.Rod1.
          v*RightWheels.Rod1.v;
        //  K = LeftWheels.K + RightWheels.K + LBar.K + RBar.K + RoBody.K;
        delta_x = IdealModel.x - Spring.RB[1];
        delta_z = IdealModel.z - Spring.RB[3];
        delta_psi = IdealModel.psi - psi;
        delta_phi = IdealModel.phi - phi_pr;
        delta_mom = IdealModel.u_psi - RoBody.M[2];
        // Control for WheelSet (phi)
        LeftJoint.Control = -RightJoint.Control;
        RightJoint.Control = omega1^2/a_phi/(cos(b_phi))^2*sin(omega1/a_phi/cos(b_phi)*time + b_phi);

        // Control for Bar (psi)
        CJoint.Control = -omega_r^2*a_psi*sin(omega_r*time + b_psi);

        theta = Modelica.Math.asin((LBar.T*{1,0,0})*{0,0,-1});
        psi = Modelica.Math.asin((RoBody.T*{1,0,0})*{0,0,-1});
        phi_pr = Modelica.Math.asin((RightWheels.Rod1.T*{0,0,1})*(RBar.T*{1,0,0}));
        eps_psi = (RoBody.OutPort.epsilon - LBar.OutPort.epsilon)*RightJoint.nAi;
        om_psi = (RoBody.OutPort.omega - LBar.OutPort.omega)*RightJoint.nAi;
        eps_phi = (RightWheels.Rod1.OutPort.epsilon - RBar.OutPort.epsilon)*RightJoint.
          nAi;
        om_phi = (RightWheels.Rod1.OutPort.omega - RBar.OutPort.omega)*RightJoint.nAi;
        connect(LeftWheels.OutPortBase, FLoor.OutPort) 
          annotation (points=[-32,-80; 0,-80; 0,-72]);
        connect(RightWheels.OutPortBase, FLoor.OutPort) 
          annotation (points=[32,-80; 0,-80; 0,-72]);
        connect(LBar.InPortJoint,CJoint. OutPortA) 
          annotation (points=[-50,38; -50,94; -6,94; -6,88]);
        connect(CJoint.InPortB,RoBody. OutPort) 
          annotation (points=[6,52; 6,46; 50,46; 50,52]);
        connect(Spring.OutPortB,RBar. InPortJointA) annotation (points=[6,38; 6,44;
              42,44; 42,38], style(color=3, rgbcolor={0,0,255}));
        connect(LBar.InPortJointB,Spring. OutPortA) annotation (points=[-40,38; -40,
              44; -6,44; -6,38],   style(color=3, rgbcolor={0,0,255}));
        connect(Spring.InPortA,LBar. OutPort) annotation (points=[-6,2; -6,-4; -50,-4;
              -50,2], style(color=3, rgbcolor={0,0,255}));
        connect(Spring.InPortB,RBar. OutPort) annotation (points=[6,2; 6,-4; 50,-4;
              50,2], style(color=3, rgbcolor={0,0,255}));
        connect(CJoint.OutPortB, RoBody.InPort) annotation (points=[6,88; 6,94; 50,94;
              50,88], style(color=3, rgbcolor={0,0,255}));
        connect(RightJoint.OutPortA, RightWheels.InPortJoint) annotation (points=[68,
              -34; 76,-34; 76,-74; 68,-74], style(color=3, rgbcolor={0,0,255}));
        connect(RightJoint.InPortA, RightWheels.OutPortJoint) annotation (points=[32,
              -34; 24,-34; 24,-68; 32,-68], style(color=3, rgbcolor={0,0,255}));
        connect(LeftJoint.InPortA, LeftWheels.OutPortJoint) annotation (points=[-32,
              -34; -24,-34; -24,-68; -32,-68], style(color=3, rgbcolor={0,0,255}));
        connect(LeftJoint.OutPortA, LeftWheels.InPortJoint) annotation (points=[-68,
              -34; -76,-34; -76,-74; -68,-74], style(color=3, rgbcolor={0,0,255}));
        connect(LBar.OutPort, LeftJoint.InPortB) annotation (points=[-50,2; -50,-4;
              -24,-4; -24,-22; -32,-22], style(color=3, rgbcolor={0,0,255}));
        connect(CJoint.InPortA, LBar.OutPort) annotation (points=[-6,52; -6,48; -24,
              48; -24,-4; -50,-4; -50,2], style(color=3, rgbcolor={0,0,255}));
        connect(RBar.OutPort, RightJoint.InPortB) annotation (points=[50,2; 50,-4; 24,
              -4; 24,-22; 32,-22], style(color=3, rgbcolor={0,0,255}));
        connect(RBar.InPortJointB, RightJoint.OutPortB) annotation (points=[58,38; 58,
              44; 76,44; 76,-22; 68,-22], style(color=3, rgbcolor={0,0,255}));
        connect(LBar.InPortJointA, LeftJoint.OutPortB) annotation (points=[-60,38;
              -60,44; -76,44; -76,-22; -68,-22], style(color=3, rgbcolor={0,0,255}));
      end TestSnakeboardSectioned;

      model IdealModel
        parameter SI.Mass m = 6;
        parameter SI.Length l = 0.2;
        parameter SI.MomentOfInertia J = 0.016;
        parameter SI.MomentOfInertia Jr = 0.072;
        parameter SI.MomentOfInertia Jw = 0.0013;
        parameter SI.Angle a_psi = 0.7;
        parameter SI.Angle beta_psi = 0;
        parameter SI.AngularVelocity omega_psi = 3;
        parameter SI.Angle a_f = 1;
        parameter SI.Angle beta_f = 0;
        parameter SI.AngularVelocity omega_f = 2;
        parameter SI.Angle a_b = -1;
        parameter SI.Angle beta_b = 0;
        parameter SI.AngularVelocity omega_b = 2;
        SI.Angle psi;
        SI.Angle phi_b;
        SI.Angle phi_f;
        SI.Angle theta;
        SI.AngularVelocity dtheta;
        SI.AngularVelocity dphi_b(start = a_b*omega_b*cos(beta_b));
        SI.AngularVelocity dphi_f(start = a_f*omega_f*cos(beta_f));
        SI.AngularVelocity dphi_fa;
        SI.AngularVelocity dpsi(start = a_psi*omega_psi*cos(beta_psi));
        SI.AngularVelocity dpsi_a;
        SI.AngularAcceleration ddtheta;
        SI.AngularAcceleration ddphi_b;
        SI.AngularAcceleration ddphi_f;
        SI.AngularAcceleration ddphi_fa;
        SI.AngularAcceleration ddpsi;
        SI.AngularAcceleration ddpsi_a;
        SI.Torque u_psi;
        SI.Torque u_f;
        SI.Torque u_b;
        SI.Torque M;
        SI.Torque BoardM;
        SI.Position x;
        SI.Position y;
        SI.Position z;
        SI.Velocity vx;
        SI.Velocity vy;
        SI.Acceleration ax;
        SI.Acceleration ay;
        SI.Force lambda_f;
        SI.Force lambda_b;
        SI.Force X;
        SI.Force Y;
        SI.Force F_fn;
        SI.Force F_bn;
      equation
        z = -y;
        der(psi) = dpsi;
        der(phi_f) = dphi_f;
        der(phi_b) = dphi_b;
        der(theta) = dtheta;
        der(dpsi) = ddpsi;
        der(dphi_f) = ddphi_f;
        der(dphi_b) = ddphi_b;
        der(dtheta) = ddtheta;
        der(x) = vx;
        der(y) = vy;
        der(vx) = ax;
        der(vy) = ay;
      //  m*ax - lambda_b*sin(phi_b + theta) - lambda_f*sin(phi_f + theta) = cos(theta);
      //  m*ay + lambda_b*cos(phi_b + theta) + lambda_f*cos(phi_f + theta) = sin(theta);
        m*ax - lambda_b*sin(phi_b + theta) - lambda_f*sin(phi_f + theta) = 0;
        m*ay + lambda_b*cos(phi_b + theta) + lambda_f*cos(phi_f + theta) = 0;
        (J + Jr + 2*Jw)*ddtheta + Jr*ddpsi + Jw*(ddphi_f + ddphi_b) - lambda_b*l*cos(phi_b) + lambda_f*l*cos(phi_f) = 0;
        Jr*(ddpsi + ddtheta) = u_psi;
        Jw*(ddphi_f + ddtheta) = u_f;
        Jw*(ddphi_b + ddtheta) = u_b;
        -sin(phi_b + theta)*vx + cos(phi_b + theta)*vy - l*cos(phi_b)*dtheta = 0;
        -sin(phi_f + theta)*vx + cos(phi_f + theta)*vy + l*cos(phi_f)*dtheta = 0;
        ddpsi = -a_psi*omega_psi^2*sin(omega_psi*time + beta_psi);
      //  ddphi_f = 0;
        ddphi_f = -a_f*omega_f^2*sin(omega_f*time + beta_f);
        ddphi_b = -a_b*omega_b^2*sin(omega_b*time + beta_b);
        dphi_fa = dtheta + dphi_f;
        ddphi_fa = ddtheta + ddphi_f;
        dpsi_a = dtheta + dpsi;
        ddpsi_a = ddtheta + ddpsi;
        M = lambda_b*l*cos(phi_b) - lambda_f*l*cos(phi_f);
        BoardM = J*ddtheta;
        X = m*ax;
        Y = m*ay;
        F_bn = lambda_b;
        F_fn = lambda_f;
        annotation (
          Diagram,
          experiment(
            StopTime=25,
            NumberOfIntervals=8000,
            Tolerance=1e-012),
          experimentSetupOutput,
          DymolaStoredErrors);
      end IdealModel;

      model TestSnakeboard
        // With Rotor and Control of phi_f, phi_b, psi
        parameter SI.Acceleration g = 9.81;
        parameter SI.Acceleration[3] Gravity = {0, -g, 0};
        parameter SI.Mass m_bar = 1;
        parameter SI.Mass m_rot = 1;
        parameter SI.Mass m_disc = 1;
        parameter SI.Mass m_rod = 1;
        parameter SI.Mass m = 2*LeftWheels.m + m_bar + m_rot;
        // Radius of Disc
        parameter SI.Radius r1 = 0.4;
        // Radius of Rod
        parameter SI.Radius r2 = 0.15;
        // Radius of Rotor
        parameter SI.Radius r3 = 0.5;
        // Horizontal Length of Rod
        parameter SI.Length L = 1.5;
        // Length of Bar
        parameter SI.Length LD = 5;
        // Width of Bar
        parameter SI.Length WD = 0.5;
        // Height of Bar
        parameter SI.Length HD = 0.2;
        // Height of Rotor
        parameter SI.Length HR = 0.4;
        parameter SI.Length h = 0.2 + r2 + HD/2;
        parameter SI.Length hr = r1 + h + HD/2;
        parameter SI.AngularVelocity omega0 = 0;
        parameter SI.Position[3] r0 = {0, r1, 0};
        parameter SI.Velocity[3] v0 = {V0, 0, 0};
        parameter SI.Velocity V0 = 0;

        // "Drive" gait (1, 1, 1)
      //  parameter SI.Angle a_psi = 0.7;
      //  parameter Real beta_psi = 0;
      //  parameter Real omega_psi = 1;
      //  parameter SI.Angle a_f = 0.3;
      //  parameter Real beta_f = 0;
      //  parameter Real omega_f = 1;
      //  parameter SI.Angle a_b = -0.3;
      //  parameter Real beta_b = 0;
      //  parameter Real omega_b = 1;

        // "Rotate" gait (2, 1, 1)
      //  parameter SI.Angle a_psi = 1;
      //  parameter Real beta_psi = 0;
      //  parameter Real omega_psi = 2;
      //  parameter SI.Angle a_f = 1;
      //  parameter Real beta_f = 0;
      //  parameter Real omega_f = 1;
      //  parameter SI.Angle a_b = -1;
      //  parameter Real beta_b = 0;
      //  parameter Real omega_b = 1;

        // "Parking" gait (3, 2, 2)
        parameter SI.Angle a_psi = 1;
        parameter Real beta_psi = 0;
        parameter Real omega_psi = 3;
        parameter SI.Angle a_f = 1;
        parameter Real beta_f = 0;
        parameter Real omega_f = 2;
        parameter SI.Angle a_b = -1;
        parameter Real beta_b = 0;
        parameter Real omega_b = 2;

        parameter Stiffness c = 1000;
        parameter Viscosity d = 5000;
        parameter Real SmallPar = 0.0000001;
        RollingWheelSet LeftWheels(
          SmallPar = SmallPar,
          m_disc = m_disc,
          m_rod = m_rod,
          Gravity = Gravity,
          c = c,
          d = d,
          r1 = r1,
          r2 = r2,
          L = L,
          r0 = r0 + {-0.5*LD + r2, 0, 0},
          v0 = v0 + cross({0, omega0, 0}, {-0.5*LD + r2, 0, 0}),
          omega1 = omega0 + a_b*omega_b*cos(beta_b),
          phi = a_b*sin(beta_b)) 
            annotation (extent=[-70,-94; -30,-54], rotation=90);
      //    phi = -atan((0.5*LD - r2)/V0*omega0) + a_b*sin(beta_b)
        RollingWheelSet RightWheels(
          SmallPar = SmallPar,
          m_disc = m_disc,
          m_rod = m_rod,
          Gravity=Gravity,
          c = c,
          d = d,
          r1 = r1,
          r2 = r2,
          L = L,
          r0 = r0 + {0.5*LD - r2, 0, 0},
          v0 = v0 + cross({0, omega0, 0}, {0.5*LD - r2, 0, 0}),
          omega1 = omega0 + a_f*omega_f*cos(beta_f),
          phi = a_f*sin(beta_f)) 
            annotation (extent=[70,-94; 30,-54], rotation=90);
      //    phi = atan((0.5*LD - r2)/V0*omega0) + a_f*sin(beta_f)
        LeftBar LBar(
          Gravity = Gravity,
          LD = LD,
          WD = WD,
          HD = HD,
          q(start = {1, 0, 0, 0}),
          omega(start = {0, omega0, 0}),
          m = 0.5*m_bar,
          I = 0.5*m_bar/12*[WD^2 + HD^2,                0,                0;
                                      0, 0.25*LD^2 + WD^2,                0;
                                      0,                0, 0.25*LD^2 + HD^2],
          r(start = {-0.25*LD, r1 + h, 0}),
          v(start = v0 + cross({0, omega0, 0}, {-LD/4, 0, 0}))) 
          annotation (extent=[-70,0; -30,40]);
        RightBar RBar(
          Gravity = Gravity,
          q(start = {1, 0, 0, 0}),
          m = 0.5*m_bar,
          I = 0.5*m_bar/12*[WD^2 + HD^2,                0,                0;
                                      0, 0.25*LD^2 + WD^2,                0;
                                      0,                0, 0.25*LD^2 + HD^2],
          r(start = {0.25*LD, r1 + h, 0}),
          v(start = v0 + cross({0, omega0, 0}, {LD/4, 0, 0})),
          omega(start = {0, omega0, 0})) annotation (extent=[30,0; 70,40]);
        Base FLoor annotation (extent=[-20,-74; 20,-34]);
        FixedServoJoint LeftJoint(
          dphi(start = -a_b*omega_b*cos(beta_b)),
          nA = {0, 1, 0},
          nB = {0, 1, 0},
          rA = {0, h, 0},
          rB = {-0.25*LD + r2, 0, 0}) annotation (extent=[-70,-48; -30,-8],  rotation=90);
        FixedServoJoint RightJoint(
          dphi(start = -a_f*omega_f*cos(beta_f)),
          nA = {0, 1, 0},
          nB = {0, 1, 0},
          rA = {0,h,0},
          rB = {0.25*LD - r2, 0, 0}) annotation (extent=[70,-48; 30,-8],  rotation=90);
        FixedServoJoint CJoint(
          dphi(start = a_psi*omega_psi*cos(beta_psi)),
          nA = {0, 1, 0},
          nB = {0, 1, 0},
          rB = {0, 0, 0},
          rA = {LD/4, HD/2, 0}) annotation (extent=[-20,50; 20,90]);
        Rotor RoBody(
          Gravity = Gravity,
          m = m_rot,
          r3 = r3,
          HR = HR,
          q(start = {cos(a_psi*sin(beta_psi)/2),0,sin(a_psi*sin(beta_psi)/2),0}),
          I = m_rot*[0.25*r3^2,        0,         0;
                             0, 0.5*r3^2,         0;
                             0,        0, 0.25*r3^2],
          v(start = v0),
          omega(start = {0, omega0 + a_psi*omega_psi*cos(beta_psi), 0}),
          r(start = {0, hr, 0})) annotation (extent=[30,50; 70,90]);
        SpringJoint Spring(
          c = c,
          d = d,
          nA = {1, 0, 0},
          nB = {1, 0, 0},
          rA = {LD/4, 0, 0},
          rB = {-LD/4, 0, 0}) annotation (extent=[-20,0; 20,40]);
        annotation (
          Diagram,
          experiment(
            StopTime=50,
            NumberOfIntervals=30000,
            Tolerance=1e-012),
          experimentSetupOutput,
          DymolaStoredErrors);
      //  Test_Kuleshov IdealModel(
      //    J=LBar.I[2, 2],
      //    Jr=RoBody.I[2, 2],
      //    L=LD/2,
      //    m=m_bar + m_rot + 2*(2*LeftWheels.m_disc + LeftWheels.m_rod),
      //    Jp=LeftWheels.Rod1.I[2, 2] + 2*(LeftWheels.Disc1.I[2, 2] + LeftWheels.m_disc*L
      //        ^2/4),
      //    a_psi=a_psi,
      //    b_psi=beta_psi,
      //    w0_psi=omega_psi,
      //    a_phi=a_f,
      //    b_phi=beta_f,
      //    w0_phi=omega_f) annotation (extent=[-96,50; -56,90]);
        IdealModel IdealModel1(
          dtheta(start = omega0),
          vx(start = V0),
          m = m,
          J = 2*(LBar.I[2, 2] + LBar.m*LD^2/16),
          Jr = RoBody.I[2, 2],
          Jw = LeftWheels.Rod1.I[2, 2] + 2*(LeftWheels.Disc1.I[2, 2] + LeftWheels.Disc1.m*LeftWheels.L^2/4),
          l = 0.5*LD - r2,
          a_psi = a_psi,
          beta_psi = beta_psi,
          omega_psi = omega_psi,
          a_f = a_f,
          beta_f = beta_f,
          omega_f = omega_f,
          a_b = a_b,
          beta_b = beta_b,
          omega_b = omega_b) 
            annotation (extent=[-94,50; -54,90]);
      //    phi_b(start = -atan((0.5*LD - r2)/(V0/omega0))),
      //    phi_f(start = atan((0.5*LD - r2)/(V0/omega0))),
        SI.Angle theta;
        SI.Angle psi;
        SI.Angle phi_pr;
        SI.AngularAcceleration eps_psi;
        SI.AngularVelocity om_psi;
        SI.AngularAcceleration eps_phi;
        SI.AngularVelocity om_phi;
      //  SI.Position delta_x;
      //  SI.Position delta_z;
      //  SI.Angle delta_psi;
      //  SI.Angle delta_phi;
      //  SI.Torque delta_mom;
        SI.Energy Ed;
      //  SI.Energy Kt;
        SI.Energy Kr;
        SI.Torque M;
        SI.Torque BoardM;
        SI.Force X;
        SI.Force Y;
        SI.Force F_bt;
        SI.Force F_bn;
        SI.Force F_ft;
        SI.Force F_fn;
        SI.Position Delta_x;
      equation
        Ed = LeftWheels.Ed + RightWheels.Ed + Spring.Ed;
        //  K = 0.5*IdealModel.Jp*LeftWheels.Rod1.omega*LeftWheels.Rod1.omega +
        //      0.5*IdealModel.Jp*RightWheels.Rod1.omega*RightWheels.Rod1.omega;
        //  Kt = 0.5*(LeftWheels.Disc1.m + LeftWheels.Disc2.m + LeftWheels.Rod1.m +
        //      RightWheels.Disc1.m + RightWheels.Disc2.m + RightWheels.Rod1.m +
        //      LBar.m + RBar.m + RoBody.m)*RoBody.v*RoBody.v;
      //  Kt = LBar.K + RBar.K + RoBody.K + 0.5*IdealModel.Jp*LeftWheels.Rod1.
      //    omega*LeftWheels.Rod1.omega + 0.5*IdealModel.Jp*RightWheels.Rod1.omega*
      //    RightWheels.Rod1.omega;
        Kr = 0.5*LeftWheels.Disc1.I[3, 3]*LeftWheels.Disc1.omega[3]*LeftWheels.Disc1.
          omega[3] + 0.5*LeftWheels.Disc2.I[3, 3]*LeftWheels.Disc2.omega[3]*LeftWheels.
           Disc2.omega[3] + 0.5*RightWheels.Disc1.I[3, 3]*RightWheels.Disc1.omega[3]*
          RightWheels.Disc1.omega[3] + 0.5*RightWheels.Disc2.I[3, 3]*RightWheels.Disc2.
          omega[3]*RightWheels.Disc2.omega[3] + 0.5*(LeftWheels.Rod1.m + LeftWheels.
          Disc1.m + LeftWheels.Disc2.m)*LeftWheels.Rod1.v*LeftWheels.Rod1.v + 0.5*(
          RightWheels.Rod1.m + RightWheels.Disc1.m + RightWheels.Disc2.m)*RightWheels.Rod1.
          v*RightWheels.Rod1.v;
        //  K = LeftWheels.K + RightWheels.K + LBar.K + RBar.K + RoBody.K;
      //  delta_x = IdealModel.x - Spring.RB[1];
      //  delta_z = IdealModel.z - Spring.RB[3];
      //  delta_psi = IdealModel.psi - psi;
      //  delta_phi = IdealModel.phi - phi_pr;
      //  delta_mom = IdealModel.u_psi - RoBody.M[2];
        // Control for WheelSet (phi)
        LeftJoint.Control = omega_b^2*a_b*sin(omega_b*time + beta_b); // A is wheelset
        RightJoint.Control = omega_f^2*a_f*sin(omega_f*time + beta_f); // A is wheelset

        // Control for Bar (psi)
        CJoint.Control = -omega_psi^2*a_psi*sin(omega_psi*time + beta_psi);

        theta = Modelica.Math.asin((LBar.T*{1,0,0})*{0,0,-1});
        psi = Modelica.Math.asin((RoBody.T*{1,0,0})*{0,0,-1});
        phi_pr = Modelica.Math.asin((RightWheels.Rod1.T*{0,0,1})*(RBar.T*{1,0,0}));
        eps_psi = (RoBody.OutPort.epsilon - LBar.OutPort.epsilon)*RightJoint.nAi;
        om_psi = (RoBody.OutPort.omega - LBar.OutPort.omega)*RightJoint.nAi;
        eps_phi = (RightWheels.Rod1.OutPort.epsilon - RBar.OutPort.epsilon)*RightJoint.
          nAi;
        om_phi = (RightWheels.Rod1.OutPort.omega - RBar.OutPort.omega)*RightJoint.nAi;
        M = LeftWheels.Disc1.M[2] + LeftWheels.Disc2.M[2] + LeftWheels.Rod1.M[2] + RightWheels.Disc1.M[2] + RightWheels.Disc2.M[2] + RightWheels.Rod1.M[2] +
              LBar.M[2] + RBar.M[2] + RoBody.M[2];
        BoardM = LBar.M[2] + RBar.M[2] + RoBody.M[2];
        X = LeftWheels.Disc1.InPortRoll.F[1] + LeftWheels.Disc2.InPortRoll.F[1] + RightWheels.Disc1.InPortRoll.F[1] + RightWheels.Disc2.InPortRoll.F[1];
        -Y = LeftWheels.Disc1.InPortRoll.F[3] + LeftWheels.Disc2.InPortRoll.F[3] + RightWheels.Disc1.InPortRoll.F[3] + RightWheels.Disc2.InPortRoll.F[3];
        F_bt = (LeftWheels.Disc1.InPortRoll.F + LeftWheels.Disc2.InPortRoll.F)*LeftWheels.Disc_on_Plane1.InPortB.T*LeftWheels.Disc_on_Plane1.tau;
        F_bn = (LeftWheels.Disc1.InPortRoll.F + LeftWheels.Disc2.InPortRoll.F)*LeftWheels.Joint1.nAi;
        F_ft = (RightWheels.Disc1.InPortRoll.F + RightWheels.Disc2.InPortRoll.F)*RightWheels.Disc_on_Plane1.InPortB.T*RightWheels.Disc_on_Plane1.tau;
        F_fn = (RightWheels.Disc1.InPortRoll.F + RightWheels.Disc2.InPortRoll.F)*RightWheels.Joint1.nAi;
        Delta_x = RoBody.r[1] - IdealModel1.x;
        connect(LeftWheels.OutPortBase, FLoor.OutPort) 
          annotation (points=[-32,-80; 0,-80; 0,-72]);
        connect(RightWheels.OutPortBase, FLoor.OutPort) 
          annotation (points=[32,-80; 0,-80; 0,-72]);
        connect(LBar.InPortJoint,CJoint. OutPortA) 
          annotation (points=[-50,38; -50,94; -6,94; -6,88]);
        connect(CJoint.InPortB,RoBody. OutPort) 
          annotation (points=[6,52; 6,46; 50,46; 50,52]);
        connect(Spring.OutPortB,RBar. InPortJointA) annotation (points=[6,38; 6,44;
              42,44; 42,38], style(color=3, rgbcolor={0,0,255}));
        connect(LBar.InPortJointB,Spring. OutPortA) annotation (points=[-40,38; -40,
              44; -6,44; -6,38],   style(color=3, rgbcolor={0,0,255}));
        connect(Spring.InPortA,LBar. OutPort) annotation (points=[-6,2; -6,-4; -50,-4;
              -50,2], style(color=3, rgbcolor={0,0,255}));
        connect(Spring.InPortB,RBar. OutPort) annotation (points=[6,2; 6,-4; 50,-4;
              50,2], style(color=3, rgbcolor={0,0,255}));
        connect(CJoint.OutPortB, RoBody.InPort) annotation (points=[6,88; 6,94; 50,94;
              50,88], style(color=3, rgbcolor={0,0,255}));
        connect(RightJoint.OutPortA, RightWheels.InPortJoint) annotation (points=[68,
              -34; 76,-34; 76,-74; 68,-74], style(color=3, rgbcolor={0,0,255}));
        connect(RightJoint.InPortA, RightWheels.OutPortJoint) annotation (points=[32,
              -34; 24,-34; 24,-68; 32,-68], style(color=3, rgbcolor={0,0,255}));
        connect(LeftJoint.InPortA, LeftWheels.OutPortJoint) annotation (points=[-32,
              -34; -24,-34; -24,-68; -32,-68], style(color=3, rgbcolor={0,0,255}));
        connect(LeftJoint.OutPortA, LeftWheels.InPortJoint) annotation (points=[-68,
              -34; -76,-34; -76,-74; -68,-74], style(color=3, rgbcolor={0,0,255}));
        connect(LBar.OutPort, LeftJoint.InPortB) annotation (points=[-50,2; -50,-4;
              -24,-4; -24,-22; -32,-22], style(color=3, rgbcolor={0,0,255}));
        connect(CJoint.InPortA, LBar.OutPort) annotation (points=[-6,52; -6,48; -24,
              48; -24,-4; -50,-4; -50,2], style(color=3, rgbcolor={0,0,255}));
        connect(RBar.OutPort, RightJoint.InPortB) annotation (points=[50,2; 50,-4; 24,
              -4; 24,-22; 32,-22], style(color=3, rgbcolor={0,0,255}));
        connect(RBar.InPortJointB, RightJoint.OutPortB) annotation (points=[58,38; 58,
              44; 76,44; 76,-22; 68,-22], style(color=3, rgbcolor={0,0,255}));
        connect(LBar.InPortJointA, LeftJoint.OutPortB) annotation (points=[-60,38;
              -60,44; -76,44; -76,-22; -68,-22], style(color=3, rgbcolor={0,0,255}));
      end TestSnakeboard;

      model TestSnakeboard1
        // With Rotor and Control of phi_f, phi_b, psi
        parameter SI.Acceleration g = 9.81;
        parameter SI.Acceleration[3] Gravity = {0, -g, 0};
        parameter SI.Mass m_bar = 1;
        parameter SI.Mass m_rot = 1;
        parameter SI.Mass m_disc = 0.000001;
        parameter SI.Mass m_rod = 1;
        parameter SI.Mass m = 2*LeftWheels.m + m_bar + m_rot;
        // Radius of Disc
        parameter SI.Radius r1 = 0.4;
        // Radius of Rod
        parameter SI.Radius r2 = 0.15;
        // Radius of Rotor
        parameter SI.Radius r3 = 0.5;
        // Horizontal Length of Rod
        parameter SI.Length L = 1.5;
        // Length of Bar
        parameter SI.Length LD = 5;
        // Width of Bar
        parameter SI.Length WD = 0.5;
        // Height of Bar
        parameter SI.Length HD = 0.2;
        // Height of Rotor
        parameter SI.Length HR = 0.4;
        parameter SI.Length h = 0.2 + r2 + HD/2;
        parameter SI.Length hr = r1 + h + HD/2;
        parameter SI.AngularVelocity omega0 = 1;
        parameter SI.Position[3] r0 = {0, r1, 0};
        parameter SI.Velocity[3] v0 = {V0, 0, 0};
        parameter SI.Velocity V0 = 1;
        // "Drive" gait (1, 1, 1)
      //  parameter SI.Angle a_psi = 0.7;
      //  parameter Real beta_psi = 0;
      //  parameter Real omega_psi = 1;
      //  parameter SI.Angle a_f = 0.3;
      //  parameter Real beta_f = 0;
      //  parameter Real omega_f = 1;
      //  parameter SI.Angle a_b = -0.3;
      //  parameter Real beta_b = 0;
      //  parameter Real omega_b = 1;
        // "Rotate" gait (2, 1, 1)
      //  parameter SI.Angle a_psi = 1;
      //  parameter Real beta_psi = 0;
      //  parameter Real omega_psi = 2;
      //  parameter SI.Angle a_f = 1;
      //  parameter Real beta_f = 0;
      //  parameter Real omega_f = 1;
      //  parameter SI.Angle a_b = -1;
      //  parameter Real beta_b = 0;
      //  parameter Real omega_b = 1;
        // "Parking" gait (3, 2, 2)
        parameter SI.Angle a_psi = 0;
      //  parameter SI.Angle a_psi = 1;
        parameter Real beta_psi = 0;
        parameter Real omega_psi = 3;
        parameter SI.Angle a_f = 0;
      //  parameter SI.Angle a_f = 1;
        parameter Real beta_f = 0;
        parameter Real omega_f = 2;
        parameter SI.Angle a_b = 0;
      //  parameter SI.Angle a_b = -1;
        parameter Real beta_b = 0;
        parameter Real omega_b = 2;
        parameter Stiffness c = 1000;
        parameter Viscosity d = 5000;
        RollingWheelSet LeftWheels(
          m_disc = m_disc,
          m_rod = m_rod,
          Gravity = Gravity,
          c = c,
          d = d,
          r1 = r1,
          r2 = r2,
          L = L,
          r0 = r0 + {-0.5*LD + r2, 0, 0},
          v0 = v0 + cross({0, omega0, 0}, {-0.5*LD + r2, 0, 0}),
          phi = -atan((0.5*LD - r2)/V0*omega0),
          omega1 = omega0) 
            annotation (extent=[-70,-94; -30,-54], rotation=90);
      //    phi = 0,
        RollingWheelSet RightWheels(
          m_disc = m_disc,
          m_rod = m_rod,
          Gravity=Gravity,
          c = c,
          d = d,
          r1 = r1,
          r2 = r2,
          L = L,
          r0 = r0 + {0.5*LD - r2,0,0},
          v0 = v0 + cross({0,omega0,0}, {0.5*LD - r2,0,0}),
          phi = atan((0.5*LD - r2)/(V0/omega0)),
          omega1 = omega0) 
            annotation (extent=[70,-94; 30,-54], rotation=90);
      //    omega1 = omega0 + a_f*omega_f*cos(beta_f))
      //    phi = 0.01,
      //    omega1 = 1)
        LeftBar LBar(
          Gravity = Gravity,
          LD = LD,
          WD = WD,
          HD = HD,
          q(start = {1, 0, 0, 0}),
          omega(start = {0, omega0, 0}),
          m = 0.5*m_bar,
          I = 0.5*m_bar/12*[WD^2 + HD^2,                0,                0;
                                      0, 0.25*LD^2 + WD^2,                0;
                                      0,                0, 0.25*LD^2 + HD^2],
          r(start = {-0.25*LD, r1 + h,0}),
          v(start = v0 + cross({0, omega0, 0}, {-LD/4, 0, 0}))) 
          annotation (extent=[-70,0; -30,40]);
        RightBar RBar(
          Gravity = Gravity,
          q(start = {1, 0, 0, 0}),
          m = 0.5*m_bar,
          I = 0.5*m_bar/12*[WD^2 + HD^2,                0,                0;
                                      0, 0.25*LD^2 + WD^2,                0;
                                      0,                0, 0.25*LD^2 + HD^2],
          r(start = {0.25*LD, r1 + h, 0}),
          v(start = v0 + cross({0, omega0, 0}, {LD/4, 0, 0})),
          omega(start = {0, omega0, 0})) annotation (extent=[30,0; 70,40]);
        Base FLoor annotation (extent=[-20,-74; 20,-34]);
        FixedServoJoint LeftJoint(
          dphi(start = -a_b*omega_b*cos(beta_b)),
          nA = {0, 1, 0},
          nB = {0, 1, 0},
          rA = {0, h, 0},
          rB = {-0.25*LD + r2, 0, 0}) annotation (extent=[-70,-48; -30,-8],  rotation=90);
        FixedServoJoint RightJoint(
          dphi(start = -1),
          nA = {0, 1, 0},
          nB = {0, 1, 0},
          rA = {0,h,0},
          rB = {0.25*LD - r2, 0, 0}) annotation (extent=[70,-48; 30,-8],  rotation=90);
      //    dphi(start = -a_f*omega_f*cos(beta_f)),
        FixedServoJoint CJoint(
          dphi(start = a_psi*omega_psi*cos(beta_psi)),
          nA = {0, 1, 0},
          nB = {0, 1, 0},
          rB = {0, 0, 0},
          rA = {LD/4, HD/2, 0}) annotation (extent=[-20,50; 20,90]);
        Rotor RoBody(
          Gravity = Gravity,
          m = m_rot,
          r3 = r3,
          HR = HR,
          q(start = {cos(a_psi*sin(beta_psi)/2), 0, sin(a_psi*sin(beta_psi)/2), 0}),
          I = m_rot*[0.25*r3^2,        0,         0;
                             0, 0.5*r3^2,         0;
                             0,        0, 0.25*r3^2],
          v(start = v0),
          omega(start = {0, omega0 + a_psi*omega_psi*cos(beta_psi), 0}),
          r(start = {0, hr, 0})) annotation (extent=[30,50; 70,90]);
        SpringJoint Spring(
          c = c,
          d = d,
          nA = {1, 0, 0},
          nB = {1, 0, 0},
          rA = {LD/4, 0, 0},
          rB = {-LD/4, 0, 0}) annotation (extent=[-20,0; 20,40]);
        annotation (
          Diagram,
          experiment(
            StopTime=100,
            NumberOfIntervals=30000,
            Tolerance=1e-012),
          experimentSetupOutput,
          DymolaStoredErrors);
      //  Test_Kuleshov IdealModel(
      //    J=LBar.I[2, 2],
      //    Jr=RoBody.I[2, 2],
      //    L=LD/2,
      //    m=m_bar + m_rot + 2*(2*LeftWheels.m_disc + LeftWheels.m_rod),
      //    Jp=LeftWheels.Rod1.I[2, 2] + 2*(LeftWheels.Disc1.I[2, 2] + LeftWheels.m_disc*L
      //        ^2/4),
      //    a_psi=a_psi,
      //    b_psi=beta_psi,
      //    w0_psi=omega_psi,
      //    a_phi=a_f,
      //    b_phi=beta_f,
      //    w0_phi=omega_f) annotation (extent=[-96,50; -56,90]);
        SI.Angle theta;
        SI.Angle psi;
        SI.Angle phi_pr;
        SI.AngularAcceleration eps_psi;
        SI.AngularVelocity om_psi;
        SI.AngularAcceleration eps_phi;
        SI.AngularVelocity om_phi;
      //  SI.Position delta_x;
      //  SI.Position delta_z;
      //  SI.Angle delta_psi;
      //  SI.Angle delta_phi;
      //  SI.Torque delta_mom;
        SI.Energy Ed;
      //  SI.Energy Kt;
        SI.Energy Kr;
        IdealModel IdealModel1(
          phi_b(start = 0),
          phi_f(start = 0.01),
          dphi_f(start = 1),
          dtheta(start = omega0),
          m = m,
          J = 2*LBar.I[2, 2],
          Jr = RoBody.I[2, 2],
          Jw = LeftWheels.Rod1.I[2, 2] + 2*(LeftWheels.Disc1.I[2, 2] + LeftWheels.Disc1.m*LeftWheels.L^2/4),
          l = 0.5*LD - r2,
          a_psi = a_psi,
          beta_psi = beta_psi,
          omega_psi = omega_psi,
          a_f = a_f,
          beta_f = beta_f,
          omega_f = omega_f,
          a_b = a_b,
          beta_b = beta_b,
          omega_b = omega_b) 
            annotation (extent=[-94,50; -54,90]);
      //    phi_b(start = -atan((0.5*LD - r2)/(V0/omega0))),
      //    phi_f(start = atan((0.5*LD - r2)/(V0/omega0))),
      equation
        Ed = LeftWheels.Ed + RightWheels.Ed + Spring.Ed;
        //  K = 0.5*IdealModel.Jp*LeftWheels.Rod1.omega*LeftWheels.Rod1.omega +
        //      0.5*IdealModel.Jp*RightWheels.Rod1.omega*RightWheels.Rod1.omega;
        //  Kt = 0.5*(LeftWheels.Disc1.m + LeftWheels.Disc2.m + LeftWheels.Rod1.m +
        //      RightWheels.Disc1.m + RightWheels.Disc2.m + RightWheels.Rod1.m +
        //      LBar.m + RBar.m + RoBody.m)*RoBody.v*RoBody.v;
      //  Kt = LBar.K + RBar.K + RoBody.K + 0.5*IdealModel.Jp*LeftWheels.Rod1.
      //    omega*LeftWheels.Rod1.omega + 0.5*IdealModel.Jp*RightWheels.Rod1.omega*
      //    RightWheels.Rod1.omega;
        Kr = 0.5*LeftWheels.Disc1.I[3, 3]*LeftWheels.Disc1.omega[3]*LeftWheels.Disc1.
          omega[3] + 0.5*LeftWheels.Disc2.I[3, 3]*LeftWheels.Disc2.omega[3]*LeftWheels.
           Disc2.omega[3] + 0.5*RightWheels.Disc1.I[3, 3]*RightWheels.Disc1.omega[3]*
          RightWheels.Disc1.omega[3] + 0.5*RightWheels.Disc2.I[3, 3]*RightWheels.Disc2.
          omega[3]*RightWheels.Disc2.omega[3] + 0.5*(LeftWheels.Rod1.m + LeftWheels.
          Disc1.m + LeftWheels.Disc2.m)*LeftWheels.Rod1.v*LeftWheels.Rod1.v + 0.5*(
          RightWheels.Rod1.m + RightWheels.Disc1.m + RightWheels.Disc2.m)*RightWheels.Rod1.
          v*RightWheels.Rod1.v;
        //  K = LeftWheels.K + RightWheels.K + LBar.K + RBar.K + RoBody.K;
      //  delta_x = IdealModel.x - Spring.RB[1];
      //  delta_z = IdealModel.z - Spring.RB[3];
      //  delta_psi = IdealModel.psi - psi;
      //  delta_phi = IdealModel.phi - phi_pr;
      //  delta_mom = IdealModel.u_psi - RoBody.M[2];
        // Control for WheelSet (phi)
        LeftJoint.Control = omega_b^2*a_b*sin(omega_b*time + beta_b); // A is wheelset
        RightJoint.Control = 0; // A is wheelset
      //  RightJoint.Control = omega_f^2*a_f*sin(omega_f*time + beta_f); // A is wheelset

        // Control for Bar (psi)
        CJoint.Control = -omega_psi^2*a_psi*sin(omega_psi*time + beta_psi);

        theta = Modelica.Math.asin((LBar.T*{1,0,0})*{0,0,-1});
        psi = Modelica.Math.asin((RoBody.T*{1,0,0})*{0,0,-1});
        phi_pr = Modelica.Math.asin((RightWheels.Rod1.T*{0,0,1})*(RBar.T*{1,0,0}));
        eps_psi = (RoBody.OutPort.epsilon - LBar.OutPort.epsilon)*RightJoint.nAi;
        om_psi = (RoBody.OutPort.omega - LBar.OutPort.omega)*RightJoint.nAi;
        eps_phi = (RightWheels.Rod1.OutPort.epsilon - RBar.OutPort.epsilon)*RightJoint.
          nAi;
        om_phi = (RightWheels.Rod1.OutPort.omega - RBar.OutPort.omega)*RightJoint.nAi;
        connect(LeftWheels.OutPortBase, FLoor.OutPort) 
          annotation (points=[-32,-80; 0,-80; 0,-72]);
        connect(RightWheels.OutPortBase, FLoor.OutPort) 
          annotation (points=[32,-80; 0,-80; 0,-72]);
        connect(LBar.InPortJoint,CJoint. OutPortA) 
          annotation (points=[-50,38; -50,94; -6,94; -6,88]);
        connect(CJoint.InPortB,RoBody. OutPort) 
          annotation (points=[6,52; 6,46; 50,46; 50,52]);
        connect(Spring.OutPortB,RBar. InPortJointA) annotation (points=[6,38; 6,44;
              42,44; 42,38], style(color=3, rgbcolor={0,0,255}));
        connect(LBar.InPortJointB,Spring. OutPortA) annotation (points=[-40,38; -40,
              44; -6,44; -6,38],   style(color=3, rgbcolor={0,0,255}));
        connect(Spring.InPortA,LBar. OutPort) annotation (points=[-6,2; -6,-4; -50,-4;
              -50,2], style(color=3, rgbcolor={0,0,255}));
        connect(Spring.InPortB,RBar. OutPort) annotation (points=[6,2; 6,-4; 50,-4;
              50,2], style(color=3, rgbcolor={0,0,255}));
        connect(CJoint.OutPortB, RoBody.InPort) annotation (points=[6,88; 6,94; 50,94;
              50,88], style(color=3, rgbcolor={0,0,255}));
        connect(RightJoint.OutPortA, RightWheels.InPortJoint) annotation (points=[68,
              -34; 76,-34; 76,-74; 68,-74], style(color=3, rgbcolor={0,0,255}));
        connect(RightJoint.InPortA, RightWheels.OutPortJoint) annotation (points=[32,
              -34; 24,-34; 24,-68; 32,-68], style(color=3, rgbcolor={0,0,255}));
        connect(LeftJoint.InPortA, LeftWheels.OutPortJoint) annotation (points=[-32,
              -34; -24,-34; -24,-68; -32,-68], style(color=3, rgbcolor={0,0,255}));
        connect(LeftJoint.OutPortA, LeftWheels.InPortJoint) annotation (points=[-68,
              -34; -76,-34; -76,-74; -68,-74], style(color=3, rgbcolor={0,0,255}));
        connect(LBar.OutPort, LeftJoint.InPortB) annotation (points=[-50,2; -50,-4;
              -24,-4; -24,-22; -32,-22], style(color=3, rgbcolor={0,0,255}));
        connect(CJoint.InPortA, LBar.OutPort) annotation (points=[-6,52; -6,48; -24,
              48; -24,-4; -50,-4; -50,2], style(color=3, rgbcolor={0,0,255}));
        connect(RBar.OutPort, RightJoint.InPortB) annotation (points=[50,2; 50,-4; 24,
              -4; 24,-22; 32,-22], style(color=3, rgbcolor={0,0,255}));
        connect(RBar.InPortJointB, RightJoint.OutPortB) annotation (points=[58,38; 58,
              44; 76,44; 76,-22; 68,-22], style(color=3, rgbcolor={0,0,255}));
        connect(LBar.InPortJointA, LeftJoint.OutPortB) annotation (points=[-60,38;
              -60,44; -76,44; -76,-22; -68,-22], style(color=3, rgbcolor={0,0,255}));
      end TestSnakeboard1;
    end Snakeboard;

    package OmniVehicle
      model RollerOnPlaneSlipping
        extends SlidingWithDryFriction;
        parameter Real R = 1;
        parameter Real R1 = R/sqrt(2);
        Real xi;
      equation
        xi^2 = rhoB[2]^2 + rhoB[3]^2;
        (xi + R1)^2 + rhoB[1]^2 = R^2;
        rhoA[2] = 0;
        gradfA = {0, 1, 0};
        gradfB = 2*{rhoB[1], rhoB[2]*(xi + R1)/xi, rhoB[3]*(xi + R1)/xi};
      end RollerOnPlaneSlipping;

      model SlippingRollerTest
        parameter Real R = 1;
        parameter Real R1 = R/sqrt(2);
        parameter Real omega0 = 10;
        Base base annotation (Placement(transformation(extent={{-80,-20},{-40,20}})));
        OnePortHeavyBody onePortHeavyBody(
          Gravity = {0, -1, 0},
          r(start = {0, R - R1, 0}),
          v(start = {0, 0, 0}),
          q(start = {1, 0, 0, 0}),
          omega(start = {omega0, 0, 0})) 
          annotation (Placement(transformation(extent={{40,-20},{80,20}})));
        RollerOnPlaneSlipping rollerOnPlaneSlipping(
          xi(start = R - R1),
          rhoA(start = {0, 0, 0}),
          rhoB(start = {0, -(R - R1), 0}),
          lambda(start = -2*R)) 
          annotation (Placement(transformation(extent={{-20,-20},{20,20}})));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                  -100},{100,100}}),
                               graphics),
          experiment(
            StopTime=10,
            NumberOfIntervals=50000,
            Tolerance=1e-006),
          experimentSetupOutput);
      equation
        connect(rollerOnPlaneSlipping.OutPortB, onePortHeavyBody.InPort) annotation (
            Line(
            points={{8,16},{8,40},{60,40},{60,16}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(rollerOnPlaneSlipping.InPortB, onePortHeavyBody.OutPort) annotation (
            Line(
            points={{8,-16},{8,-40},{60,-40},{60,-16}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(rollerOnPlaneSlipping.InPortA, base.OutPort) annotation (Line(
            points={{-8,-16},{-8,-40},{-60,-40},{-60,-16}},
            color={0,0,255},
            smooth=Smooth.None));
      end SlippingRollerTest;

      model RollerOverHorizontalSurface
        extends Constraint;
        parameter Real alpha = 1;
        parameter Real R = 1;
        parameter Real R1 = R*cos(alpha);
        parameter Real delta = 10^(-6);
        parameter Real fric = 0.1;
        parameter Real stiff = 100;
        parameter Real damp = 10;
        Real kappa;
        Real[3] vA;
        Real[3] vB;
        Real[3] vr;
        Real[3] vrtan;
        Real vrt;
        Real[3] RBt;
        Real RBn;
        SI.Position[3] rA;
        SI.Position[3] rB;
        Real h;
        Real[3] nA;
      equation
        h = 0;//
        rA = InPortA.r + InPortA.T*rhoA;
        rB = InPortB.r + InPortB.T*rhoB;
        nA = {0, 1, 0};
        vA = InPortA.v + cross(InPortA.omega, rA - InPortA.r);
        vB = InPortB.v + cross(InPortB.omega, rB - InPortB.r);
        vr = vB - vA;
        vrn = vr*nA;
        vrtan = vr - vrn*nA;
        vrt = sqrt(vrtan*vrtan);
        RBn = OutPortB.F*nA;
        RBn = if h >= 0 then 0 else -stiff*h*sqrt(abs(h)) - damp*vrn;
        RBt = -fric*vrtan*(if vrt <= delta then 1/delta else 1/vrt)*RBn + kappa*nA;
        OutPortB.F = RBt + RBn*nA;
        OutPortA.P = rA;
        OutPortB.P = rB;
        OutPortB.M = zeros(3);
      end RollerOverHorizontalSurface;

      partial model RollerContactTracking
        extends Constraint;
        parameter Integer n = 4 "Number of rollers";
        parameter Real alpha = Modelica.Constants.pi/n
          "Max angle of the half-sector";
        parameter Real cos_of_max = cos(Modelica.Constants.pi/2 - alpha); // cos(pi/2 - pi/n)
        parameter Real R = 1 "Omni wheel outer radius";
        parameter Real R1 = R*cos(alpha) "Omni wheel inner radius";
        parameter Real L1 = R*sin(alpha) "Half roller length";
        parameter Real[3] nA = {0, 1, 0}
          "Roller over horizontal surface: vertical unit vector";
        parameter Real[3] i = {1, 0, 0} "Roller axis of symmetry unit vector";
        parameter Real psi = 0 "Angle of roller distortion (fixed axis turn)";
        SI.Position[3] rA;
        SI.Position[3] rB;
      //  Real[3] b;
        Real[3] c;
        Real[3] d;
        Real h;
        Real j;
        Real cosBtwAxisAndVert;
      equation
      //  b = cross(InPortB.T*i, i);
        d = cross(InPortB.T*i, nA);
        cosBtwAxisAndVert = (InPortB.T*i)*nA;

        if noEvent(abs(cosBtwAxisAndVert) < cos_of_max) then
           c = R1*cross(d, InPortB.T*i)/sqrt(d*d);
           rB = InPortB.r + c - R*nA;
           j = 1; // IN CONTACT
        else
           if noEvent(cosBtwAxisAndVert > 0) then
             c = -L1*InPortB.T*i;
             j = 2; // ??? roller is to the right of vertical
           else
             c = L1*InPortB.T*i;
             j = 3; // ??? roller is to the left of vertical
           end if;
           rB = InPortB.r + c;
        end if;

        rA = {rB[1], 0, rB[3]};
        h = rB[2];
      end RollerContactTracking;

      partial model RollerContactTrackingGeneral
        extends Constraint;
        parameter Integer n = 4 "Number of rollers";
        parameter Real R = 1 "Omni wheel outer radius (ellipse small axis)";
        parameter Real psi = 1 "Angle of roller distortion (fixed axis turn)";

        parameter Real alpha = Modelica.Constants.pi/n
          "Max angle of the half-sector";
        parameter Real Q = R/cos(psi) "Ellipse large axis";
        parameter Real R1 = R*cos(alpha) "Omni wheel inner radius";
        parameter Real L1 = R*sin(alpha)
          "Half roller visible (projection) length";
        parameter Real L2 = L1/cos(psi) "Half roller length";
        parameter Real[3] nA = {0, 1, 0}
          "Roller over horizontal surface: vertical unit vector (in global coordinates)";
        parameter Real[2] gradAtRollerTip = 2*{L2/Q^2, R1/R^2};
        parameter Real[2] n_at_max = gradAtRollerTip/sqrt(gradAtRollerTip*gradAtRollerTip)
          "Normal to ellipse in local coord (it is in vert plane)";
        parameter Real[3] i = {1, 0, 0} "Roller axis of symmetry unit vector";
        parameter Real cos_of_max = 0.3820515; // For psi = 1.0
      //  parameter Real cos_of_max = 0.44; // For psi = 0.9
      //  parameter Real cos_of_max = 0.4927; // For psi = 0.8
      //  parameter Real cos_of_max = 0.5408; // For psi = 0.7
      //  parameter Real cos_of_max = 0.584; // For psi = 0.6
      //  parameter Real cos_of_max = 0.62; // For psi = 0.5
      //  parameter Real cos_of_max = 0.65128; // For psi = 0.4
      //  parameter Real cos_of_max = n_at_max * {1, 0};
      //  parameter Real cos_of_max = cos(Modelica.Constants.pi/2 - alpha); // cos(pi/2 - pi/n)
        SI.Position[3] rA; // A - plane
        SI.Position[3] rB; // B - roller

      // Auxiliary roller base:
        Real[3] ni "Roller axis in global coords";
        Real[3] crs "horizontal ni cross vertical";
        Real[3] nk "Horizontal (along crs)";

      // Auxiliary wheel base
        Real[3] n1j "In vertical direction (= nA)";
        Real[3] n1k "Horizontal wheel axis, delivered from above";
        Real lambda "Parameter to be computed";
        Real[3] rho "Unit vector between mass centers";
        Real[3] OBPB;

        Real j;
        Real cosBtwAxisAndVert;
      equation
      // Intermediate wheel coordinates:
        n1j = nA;

      // Intermediate roller coordinates:
        ni = InPortB.T*i;
        crs = cross(ni, nA);
        nk = crs/sqrt(crs*crs);
        cosBtwAxisAndVert = ni*nA;

        if noEvent(abs(cosBtwAxisAndVert) < cos_of_max and InPortB.r[2] < R) then
          lambda = (R*(n1j*nk) - R1*(rho*nk))/(n1k*nk);
          OBPB = -R*n1j + lambda*n1k + R1*rho;
          rB = InPortB.r + OBPB;
          j = 1; // IN CONTACT
        else
          lambda = R*(n1j*nk)/(n1k*nk);
          OBPB = -R*n1j;
          if noEvent(cosBtwAxisAndVert > 0) then
            rB = InPortB.r - L2*ni;
            j = 2; // ??? roller is to the right of vertical
          else
            rB = InPortB.r + L2*ni;
            j = 3; // ??? roller is to the left of vertical
          end if;
        end if;

        rA = {rB[1], 0, rB[3]};

        annotation (experiment(
            StopTime=10,
            NumberOfIntervals=50000,
            Tolerance=1e-009,
            Algorithm="Dassl"), experimentSetupOutput);
      end RollerContactTrackingGeneral;

      partial model RollerContactTrackingGeneralStep
        extends Constraint;
        parameter Integer n = 4 "Number of rollers";
        parameter Real R = 1 "Omni wheel outer radius (ellipse small axis)";
        parameter Real psi = 0 "Angle of roller distortion (fixed axis turn)";
        parameter Real cospsi = cos(psi) "Auxiliary value";

        parameter Real alpha = Modelica.Constants.pi/n
          "Max angle of the half-sector";
        parameter Real Q = R/cos(psi) "Ellipse large axis";
        parameter Real R1 = R*cos(alpha) "Omni wheel inner radius";
        parameter Real L1 = R*sin(alpha)
          "Half roller visible (projection) length";
        parameter Real L2 = L1/cos(psi) "Half roller length";
        parameter Real[3] nA = {0, 1, 0}
          "Roller over horizontal surface: vertical unit vector (in global coordinates)";
        parameter Real[2] gradAtRollerTip = 2*{L2/Q^2, R1/R^2};
        parameter Real[2] n_at_max = gradAtRollerTip/sqrt(gradAtRollerTip*gradAtRollerTip)
          "Normal to ellipse in local coord (it is in vert plane)";
        parameter Real[3] i = {1, 0, 0} "Roller axis of symmetry unit vector";
        parameter Real cos_of_max = 0.3820515; // For psi = 1.0
      //  parameter Real cos_of_max = 0.44; // For psi = 0.9
      //  parameter Real cos_of_max = 0.4927; // For psi = 0.8
      //  parameter Real cos_of_max = 0.5408; // For psi = 0.7
      //  parameter Real cos_of_max = 0.584; // For psi = 0.6
      //  parameter Real cos_of_max = 0.62; // For psi = 0.5
      //  parameter Real cos_of_max = 0.65128; // For psi = 0.4
      //  parameter Real cos_of_max = n_at_max * {1, 0};
      //  parameter Real cos_of_max = cos(Modelica.Constants.pi/2 - alpha); // cos(pi/2 - pi/n)
        SI.Position[3] rA; // A - plane
        SI.Position[3] rB; // B - roller

      // Delivered from above
        Real[3] rO "Wheel center position";
        Real[3] n1k "Horizontal wheel axis";

      // Auxiliaries
        Real[3] ni "Roller axis in global coords";
        Real[3] rho "Unit vector between wheel and roller mass centers";
        Real[3] OBPB; // Contact poit relative position
        Real h;     // point of contact vertical coordinate
        Real j;
        Real cosBtwAxisAndVert;
        Real w;
        Real cosq;
        Real sinq;
      equation
        w = abs(cosBtwAxisAndVert);

      // Intermediate roller coordinates:
        ni = InPortB.T*i;
        rho = (rO - InPortB.r)/R1; // to verify unit length
        cosq = rho*nA;
        sinq = cross(nA, rho)*n1k;

        cosBtwAxisAndVert = ni*nA;

        if noEvent(abs(cosBtwAxisAndVert) < cos_of_max and InPortB.r[2] < R) then
          OBPB = -(R1*sinq/cosq/cospsi)*ni - (R - R1/cosq)*nA;
          rB = InPortB.r + OBPB;
          j = 1; // IN CONTACT
        else
          OBPB = zeros(3); // does not matter
          if noEvent(cosBtwAxisAndVert > 0) then
            rB = InPortB.r - L2*ni;
            j = 2; // ??? roller is to the right of vertical
          else
            rB = InPortB.r + L2*ni;
            j = 3; // ??? roller is to the left of vertical
          end if;
        end if;

        rA = {rB[1], 0, rB[3]};
        h = rB[2];

        annotation (experiment(
            StopTime=10,
            NumberOfIntervals=50000,
            Tolerance=1e-009,
            Algorithm="Dassl"), experimentSetupOutput);
      end RollerContactTrackingGeneralStep;

      partial model RollerContactTrackingGeneralEllipse
        extends Constraint;
        parameter Integer n = 4 "Number of rollers";
        parameter Real R = 1 "Omni wheel outer radius (ellipse small axis)";
        parameter Real psi = 0 "Angle of roller distortion (fixed axis turn)";

        parameter Real alpha = Modelica.Constants.pi/n
          "Max angle of the half-sector";
        parameter Real Q = R/cos(psi) "Ellipse large axis";
        parameter Real R1 = R*cos(alpha) "Omni wheel inner radius";
        parameter Real L1 = R*sin(alpha)
          "Half roller visible (projection) length";
        parameter Real L2 = L1/cos(psi) "Half roller length";
        parameter Real[3] nA = {0, 1, 0}
          "Roller over horizontal surface: vertical unit vector (in global coordinates)";
        parameter Real[2] gradAtRollerTip = 2*{L2/Q^2, R1/R^2};
        parameter Real[2] n_at_max = gradAtRollerTip/sqrt(gradAtRollerTip*gradAtRollerTip)
          "Normal to ellipse in local coord (it is in vert plane)";
        parameter Real[3] i = {1, 0, 0} "Roller axis of symmetry unit vector";
        parameter Real cos_of_max = 0.65128; // For psi = 0.4
      //  parameter Real cos_of_max = 0.6512884; // For psi = 0.4
      //  parameter Real cos_of_max = n_at_max * {1, 0};
        SI.Position[3] rA; // A - plane
        SI.Position[3] rB; // B - roller

        Real[3] r_center "Ellipse center in global coords";

        // Intermediate coordinates:
        Real[3] ni "Roller axis in global coords";
        Real[3] crs "horizontal ni cross vertical";
        Real[3] nj "In vertical plane";
        Real[3] nk "Horizontal (along crs (singularity!))";
        Real[3,3] Ti "Intermediate to global transition";
        Real[3] rB_int
          "Closest point in intermed coord (from center to contact)";
        Real[3] nA_int "Vertical unit vector in intermediate coordinates";

      //  Real[3] b;
        Real h;     // point of contact vertical coordinate
        Real j;
        Real cosBtwAxisAndVert;
        Real w;
      equation
        w = abs(cosBtwAxisAndVert);

          // Intermediate coordinates:
        ni = InPortB.T*i;
        crs = cross(ni, nA);
        // nj singularity at || nA !
        Ti = [ni[1], nj[1], nk[1];
              ni[2], nj[2], nk[2];
              ni[3], nj[3], nk[3]];
        nA_int = transpose(Ti)*nA; // note that stuff doesn't work when not in contact

      //  b = cross(InPortB.T*i, i);
        cosBtwAxisAndVert = ni*nA;

        if noEvent(abs(cosBtwAxisAndVert) < cos_of_max) then
          nk = crs/sqrt(crs*crs);
          nj = cross(nk, ni);
          r_center = InPortB.r + R1*nj;
          rB_int = - {nA_int[1]*Q^2, nA_int[2]*R^2, 0} / sqrt(nA_int[1]^2*Q^2 + nA_int[2]^2*R^2);
          rB = r_center + Ti*rB_int;
          j = 1; // IN CONTACT
        else
          nk = crs; // does not matter
          nj = cross(nk, ni);
          r_center = InPortB.r + R1*nj;
          if noEvent(cosBtwAxisAndVert > 0) then
            rB_int = {-L2, -R1, 0};
            rB = InPortB.r - L2*ni;
            j = 2; // ??? roller is to the right of vertical
          else
            rB_int = {L2, -R1, 0};
            rB = InPortB.r + L2*ni;
            j = 3; // ??? roller is to the left of vertical
          end if;
        end if;

        rA = {rB[1], 0, rB[3]};
        h = rB[2];
      end RollerContactTrackingGeneralEllipse;

      model OmniWheel
        parameter Integer n = 4 "Number of rollers";
        parameter Real alpha = Modelica.Constants.pi/n
          "Max angle of the half-sector";
        parameter Real R = 1 "Omni wheel outer radius";
        parameter Real R1 = R*cos(alpha) "Omni wheel inner radius";
        parameter Real[3] v0 = {1, 0, 0};
        parameter Real[3] omega0 = {0, 0, -1};
        parameter Real pi = Modelica.Constants.pi;
        TwoPortsHeavyBody Roller0(
          Gravity = {0, -1, 0},
          r(start = {0, R - R1,0}),
          v(start = v0 + cross(omega0, {0, -R1, 0})),
          q(start = {1, 0, 0, 0}),
          omega(start = omega0)) 
          annotation (Placement(transformation(extent={{-30,-70},{-10,-50}})));
        RollerContactForces Contact0 
          annotation (Placement(transformation(extent={{-60,-70},{-40,-50}})));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                  -100},{100,100}}),
                               graphics),
          experiment(
            StopTime=40,
            NumberOfIntervals=50000,
            Tolerance=1e-008),
          experimentSetupOutput);
        RollerContactForces Contact1 
          annotation (Placement(transformation(extent={{-60,-30},{-40,-10}})));
        TwoPortsHeavyBody Roller1(
          Gravity = {0, -1, 0},
          r(start = {R1, R, 0}),
          v(start = v0 + cross(omega0, {R1, 0, 0})),
          q(start = {cos(pi/4), 0, 0, sin(pi/4)}),
          omega(start = omega0)) 
          annotation (Placement(transformation(extent={{-30,-30},{-10,-10}})));
        RollerContactForces Contact2 
          annotation (Placement(transformation(extent={{-60,10},{-40,30}})));
        TwoPortsHeavyBody Roller2(
          Gravity = {0, -1, 0},
          r(start = {0, R + R1, 0}),
          v(start = v0 + cross(omega0, {0, R1, 0})),
          q(start = {cos(pi/2), 0, 0, sin(pi/2)}),
          omega(start = omega0)) 
          annotation (Placement(transformation(extent={{-30,10},{-10,30}})));
        RollerContactForces Contact3 
          annotation (Placement(transformation(extent={{-60,50},{-40,70}})));
        TwoPortsHeavyBody Roller3(
          Gravity = {0, -1, 0},
          r(start = {-R1, R, 0}),
          v(start = v0 + cross(omega0, {-R1, 0, 0})),
          q(start = {cos(3*pi/4), 0, 0, sin(3*pi/4)}),
          omega(start = omega0)) 
          annotation (Placement(transformation(extent={{-30,50},{-10,70}})));
        FixedJoint Joint3(
          nA = {1, 0, 0},
          nB = {0, -1, 0},
          rA = {0, 0, 0},
          rB = {-R1, 0, 0}) 
          annotation (Placement(transformation(extent={{0,50},{20,70}})));
        FixedJoint Joint2(
          nA = {1, 0, 0},
          nB = {-1, 0, 0},
          rA = {0, 0, 0},
          rB = {0, R1, 0}) 
          annotation (Placement(transformation(extent={{0,10},{20,30}})));
        FixedJoint Joint1(
          nA = {1, 0, 0},
          nB = {0, 1, 0},
          rA = {0, 0, 0},
          rB = {R1, 0, 0}) 
          annotation (Placement(transformation(extent={{0,-30},{20,-10}})));
        FixedJoint Joint0(
          nA = {1, 0, 0},
          nB = {1, 0, 0},
          rA = {0, 0, 0},
          rB = {0, -R1, 0}) 
          annotation (Placement(transformation(extent={{0,-70},{20,-50}})));
        FivePortsHeavyBody Wheel(
          Gravity = {0, -1, 0},
          r(start = {0, R, 0}),
          v(start = v0),
          q(start = {1, 0, 0, 0}),
          omega(start = omega0)) 
          annotation (Placement(transformation(extent={{40,-10},{60,10}})));
        KinematicPort InPortK 
          annotation (Placement(transformation(extent={{-50,-100},{-30,-80}})));
        WrenchPort InPortF 
          annotation (Placement(transformation(extent={{30,80},{50,100}})));
        KinematicPort OutPortK 
          annotation (Placement(transformation(extent={{30,-100},{50,-80}})));
      equation
        connect(Contact0.InPortB, Roller0.OutPort) 
          annotation (Line(
            points={{-46,-68},{-46,-76},{-20,-76},{-20,-68}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(Contact0.OutPortB, Roller0.InPort) 
          annotation (Line(
            points={{-46,-52},{-46,-44},{-24,-44},{-24,-52.1}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(Contact1.InPortB, Roller1.OutPort) annotation (Line(
            points={{-46,-28},{-46,-36},{-20,-36},{-20,-28}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(Contact1.OutPortB, Roller1.InPort) annotation (Line(
            points={{-46,-12},{-46,-4},{-24,-4},{-24,-12.1}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(Contact2.InPortB, Roller2.OutPort) annotation (Line(
            points={{-46,12},{-46,4},{-20,4},{-20,12}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(Contact2.OutPortB, Roller2.InPort) annotation (Line(
            points={{-46,28},{-46,36},{-24,36},{-24,27.9}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(Contact3.InPortB, Roller3.OutPort) annotation (Line(
            points={{-46,52},{-46,44},{-20,44},{-20,52}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(Contact3.OutPortB, Roller3.InPort) annotation (Line(
            points={{-46,68},{-46,76},{-24,76},{-24,67.9}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(Roller3.InPort1, Joint3.OutPortA) annotation (Line(
            points={{-16,68},{-16,76},{6,76},{6,68}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(Roller2.InPort1, Joint2.OutPortA) annotation (Line(
            points={{-16,28},{-16,36},{6,36},{6,28}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(Roller1.InPort1, Joint1.OutPortA) annotation (Line(
            points={{-16,-12},{-16,-4},{6,-4},{6,-12}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(Roller2.OutPort, Joint2.InPortA) annotation (Line(
            points={{-20,12},{-20,4},{6,4},{6,12}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(Roller3.OutPort, Joint3.InPortA) annotation (Line(
            points={{-20,52},{-20,44},{6,44},{6,52}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(Roller0.InPort1, Joint0.OutPortA) annotation (Line(
            points={{-16,-52},{-16,-44},{6,-44},{6,-52}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(Roller0.OutPort, Joint0.InPortA) annotation (Line(
            points={{-20,-68},{-20,-76},{6,-76},{6,-68}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(Roller1.OutPort, Joint1.InPortA) annotation (Line(
            points={{-20,-28},{-20,-36},{6,-36},{6,-28}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(Wheel.InPort, Joint0.OutPortB) annotation (Line(
            points={{43.4,8},{44,8},{44,14},{36,14},{36,-44},{14,-44},{14,-52}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(Wheel.InPort2, Joint2.OutPortB) annotation (Line(
            points={{52.2,8},{52.2,36},{14,36},{14,28}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(Wheel.InPort1, Joint1.OutPortB) annotation (Line(
            points={{47.8,8},{48,8},{48,18},{32,18},{32,-4},{14,-4},{14,-12}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(Wheel.InPort3, Joint3.OutPortB) annotation (Line(
            points={{56.6,8},{56,8},{56,76},{14,76},{14,68}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(Wheel.OutPort, Joint3.InPortB) annotation (Line(
            points={{50,-8},{50,-16},{28,-16},{28,44},{14,44},{14,52}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(Wheel.OutPort, Joint2.InPortB) annotation (Line(
            points={{50,-8},{50,-16},{28,-16},{28,4},{14,4},{14,12}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(Wheel.OutPort, Joint1.InPortB) annotation (Line(
            points={{50,-8},{50,-36},{14,-36},{14,-28}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(Wheel.OutPort, Joint0.InPortB) annotation (Line(
            points={{50,-8},{50,-76},{14,-76},{14,-68}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(Wheel.InPort4, InPortF) annotation (Line(
            points={{58,0},{70,0},{70,90},{40,90}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(Wheel.OutPort, OutPortK) annotation (Line(
            points={{50,-8},{50,-76},{40,-76},{40,-90}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(Contact1.InPortA, InPortK) annotation (Line(
            points={{-54,-28},{-54,-36},{-70,-36},{-70,-90},{-40,-90}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(Contact0.InPortA, InPortK) annotation (Line(
            points={{-54,-68},{-56,-68},{-56,-90},{-40,-90}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(Contact2.InPortA, InPortK) annotation (Line(
            points={{-54,12},{-54,4},{-70,4},{-70,-90},{-40,-90}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(Contact3.InPortA, InPortK) annotation (Line(
            points={{-54,52},{-54,44},{-70,44},{-70,-90},{-40,-90}},
            color={0,0,255},
            smooth=Smooth.None));
      end OmniWheel;

      model ThreeWheelsVehicle
        OmniWheel Wheel1 
          annotation (Placement(transformation(extent={{-46,40},{-6,80}})));
        OmniWheel Wheel2 
          annotation (Placement(transformation(extent={{-46,-20},{-6,20}})));
        OmniWheel Wheel3 
          annotation (Placement(transformation(extent={{-46,-80},{-6,-40}})));
        Base Floor 
                  annotation (Placement(transformation(extent={{-96,-20},{-56,
                  20}})));
        FixedJoint Joint1(
          nA = {1, 0, 0},
          nB = {0, 1, 0},
          rA = {0, 0, 0},
          rB = {R1, 0, 0}) 
          annotation (Placement(transformation(extent={{4,40},{44,80}})));
        FixedJoint Joint2(
          nA = {1, 0, 0},
          nB = {0, 1, 0},
          rA = {0, 0, 0},
          rB = {R1, 0, 0}) 
          annotation (Placement(transformation(extent={{4,-20},{44,20}})));
        FixedJoint Joint3(
          nA = {1, 0, 0},
          nB = {0, 1, 0},
          rA = {0, 0, 0},
          rB = {R1, 0, 0}) 
          annotation (Placement(transformation(extent={{4,-80},{44,-40}})));
        ThreePortsHeavyBody Body 
          annotation (Placement(transformation(extent={{54,-20},{94,20}})));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                  -100},{100,100}}),       graphics));
      equation
        connect(Wheel1.InPortK, Floor.OutPort) annotation (Line(
            points={{-34,42},{-34,34},{-52,34},{-52,-26},{-76,-26},{-76,-16}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(Wheel2.InPortK, Floor.OutPort) annotation (Line(
            points={{-34,-18},{-34,-26},{-76,-26},{-76,-16}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(Wheel3.InPortK, Floor.OutPort) annotation (Line(
            points={{-34,-78},{-34,-86},{-52,-86},{-52,-26},{-76,-26},{-76,-16}},
            color={0,0,255},
            smooth=Smooth.None));

        connect(Wheel1.InPortF, Joint1.OutPortA) annotation (Line(
            points={{-18,78},{-18,86},{16,86},{16,76}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(Wheel2.InPortF, Joint2.OutPortA) annotation (Line(
            points={{-18,18},{-18,26},{16,26},{16,16}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(Wheel3.InPortF, Joint3.OutPortA) annotation (Line(
            points={{-18,-42},{-18,-34},{16,-34},{16,-44}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(Wheel3.OutPortK, Joint3.InPortA) annotation (Line(
            points={{-18,-78},{-18,-86},{16,-86},{16,-76}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(Wheel2.OutPortK, Joint2.InPortA) annotation (Line(
            points={{-18,-18},{-18,-26},{16,-26},{16,-16}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(Wheel1.OutPortK, Joint1.InPortA) annotation (Line(
            points={{-18,42},{-18,34},{16,34},{16,44}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(Joint2.OutPortB, Body.InPort) annotation (Line(
            points={{32,16},{32,26},{62,26},{62,16}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(Joint1.OutPortB, Body.InPort1) annotation (Line(
            points={{32,76},{32,86},{74,86},{74,16}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(Joint3.OutPortB, Body.InPort2) annotation (Line(
            points={{32,-44},{32,-34},{96,-34},{96,26},{86,26},{86,16}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(Body.OutPort, Joint2.InPortB) annotation (Line(
            points={{74,-16},{74,-26},{32,-26},{32,-16}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(Body.OutPort, Joint1.InPortB) annotation (Line(
            points={{74,-16},{74,-26},{48,-26},{48,34},{32,34},{32,44}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(Body.OutPort, Joint3.InPortB) annotation (Line(
            points={{74,-16},{74,-26},{48,-26},{48,-86},{32,-86},{32,-76}},
            color={0,0,255},
            smooth=Smooth.None));
      end ThreeWheelsVehicle;

      package PatchContact
        partial model RollerContactVelocities
          extends RollerContactTracking;
          Real kappa;
          Real[3] PA;
          Real[3] PB;
          Real[3] vPA;
          Real[3] vPB;
          Real[3] relv;
          Real relvn;
          Real vPAn;
          Real vPBn;
          Real[3] vPAt;
          Real[3] vPBt;
          Real[3] relvt;
          Real relvtsqrt;
        equation
          kappa = rB[2];

          if noEvent(kappa <= 0) then
            PA = (rB + rA)/2;
            PB = PA;
          else
            PA = rA;
            PB = rB;
          end if;
          vPA = InPortA.v + cross(InPortA.omega, PA - InPortA.r);
          vPB = InPortB.v + cross(InPortB.omega, PB - InPortB.r);
          relv = vPB - vPA;
          relvn = relv*nA;
          vPAn = vPA*nA;
          vPBn = vPB*nA;
          vPAt = vPA - vPAn*nA;
          vPBt = vPB - vPBn*nA;
          relvt = vPBt - vPAt;
          relvtsqrt = sqrt(relvt*relvt);

          OutPortA.P = PA;
          OutPortB.P = PB;

        end RollerContactVelocities;

        model RollerContactForces
          extends RollerContactVelocities;
          parameter Real delta = 10^(-6);
          parameter Real fric = 0.1;
          parameter Real stiff = 1000;
          parameter Real damp = 100;
        //  Real mu;
          Real[3] Forcet;
          Real Forcen;
          Real Forcev;
          Real w;
        equation
          w = abs((InPortB.T*i)*nA) - cos_of_max;
          if noEvent(w < 0 and kappa < 0) then
            Forcen = -stiff*kappa*sqrt(abs(kappa));
          else
            Forcen = 0;
          end if;
          Forcet = -fric*relvt*(if relvtsqrt <= delta then 1/delta else 1/relvtsqrt)*Forcen;
        //  Forcet = -fric*relvt*(if relvtsqrt <= delta then 1/delta else 1/relvtsqrt)*Forcen + mu*nA;
          if noEvent(w < 0 and kappa < 0) then
        //    Forcev =  - (if noEvent(relvn > 0) then d*(-kappa)*sqrt(relvn)*sqrt(sqrt(relvn)) else 0);
        //    Forcev =  -d*(if noEvent(relvn > 0) then (-kappa)*exp(0.75*log(relvn)) else -(-kappa)*exp(0.75*log(-relvn)));
        //    Forcev =  - (if noEvent(relvn > 0) then d*(-kappa)*sqrt(relvn) else 0);
            Forcev =  if noEvent(relvn < 0) then damp*kappa*relvn else 0;
        //    Forcev =  - (if noEvent(relvn > 0) then d*(-kappa)^2*sqrt(relvn)*sqrt(sqrt(relvn)) else 0);
          else
            Forcev = 0;
          end if;

          OutPortB.F = Forcet + Forcen*nA + Forcev*nA;
          OutPortB.M = zeros(3);
        end RollerContactForces;

        model MovingRollerTest
          parameter Integer n = 4 "Number of rollers";
          parameter Real alpha = Modelica.Constants.pi/n
            "Max angle of the half-sector";
          parameter Real R = 1 "Omni wheel outer radius";
          parameter Real R1 = R*cos(alpha) "Omni wheel inner radius";
          parameter Real omega0 = 10;
          Base base annotation (Placement(transformation(extent={{-80,-20},{-40,20}})));
          OnePortHeavyBody onePortHeavyBody(
            Gravity = {0, -1, 0},
            r(start = {0, 2, 0}),
            v(start = {0, 0, 0}),
            q(start = {1, 0, 0, 0}),
            omega(start = {omega0, 0, 0})) 
            annotation (Placement(transformation(extent={{40,-20},{80,20}})));
          RollerContactForces rollerContactForces 
            annotation (Placement(transformation(extent={{-20,-20},{20,20}})));
          annotation (Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                    -100},{100,100}}),
                                 graphics),
            experiment(
              StopTime=50,
              NumberOfIntervals=50000,
              Tolerance=1e-008),
            experimentSetupOutput);
        equation
          connect(base.OutPort, rollerContactForces.InPortA) annotation (Line(
              points={{-60,-16},{-60,-40},{-8,-40},{-8,-16}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(rollerContactForces.InPortB, onePortHeavyBody.OutPort) 
            annotation (Line(
              points={{8,-16},{8,-40},{60,-40},{60,-16}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(rollerContactForces.OutPortB, onePortHeavyBody.InPort) 
            annotation (Line(
              points={{8,16},{8,40},{60,40},{60,16}},
              color={0,0,255},
              smooth=Smooth.None));
        end MovingRollerTest;

        model AutonomousOmniWheelTest
          parameter Integer n = 4 "Number of rollers";
          parameter Real alpha = Modelica.Constants.pi/n
            "Max angle of the half-sector";
          parameter Real R = 1 "Omni wheel outer radius";
          parameter Real R1 = R*cos(alpha) "Omni wheel inner radius";
          parameter Real[3] r0 = {0, R, 0};
          parameter Real[4] q0 = {1, 0, 0, 0};
          parameter Real[3] v0 = {1, 0, 0};
          parameter Real[3] omega0 = {0, 0, -1};
          parameter Real pi = Modelica.Constants.pi;
          Base Floor 
                    annotation (Placement(transformation(extent={{-90,-10},{-70,10}})));
          TwoPortsHeavyBody Roller0(
            Gravity = {0, -1, 0},
            r(start = {0, R - R1,0}),
            v(start = v0 + cross(omega0, {0, -R1, 0})),
            q(start = {1, 0, 0, 0}),
            omega(start = omega0)) 
            annotation (Placement(transformation(extent={{-20,-70},{0,-50}})));
          RollerContactForces Contact0 
            annotation (Placement(transformation(extent={{-60,-70},{-40,-50}})));
          annotation (Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                    -100},{100,100}}),
                                 graphics),
            experiment(
              StopTime=20,
              NumberOfIntervals=50000,
              Tolerance=1e-008),
            experimentSetupOutput);
          RollerContactForces Contact1 
            annotation (Placement(transformation(extent={{-60,-30},{-40,-10}})));
          TwoPortsHeavyBody Roller1(
            Gravity = {0, -1, 0},
            r(start = {R1, R, 0}),
            v(start = v0 + cross(omega0, {R1, 0, 0})),
            q(start = {cos(pi/4), 0, 0, sin(pi/4)}),
            omega(start = omega0 + {1, 0, 0})) 
            annotation (Placement(transformation(extent={{-20,-30},{0,-10}})));
          RollerContactForces Contact2 
            annotation (Placement(transformation(extent={{-60,10},{-40,30}})));
          TwoPortsHeavyBody Roller2(
            Gravity = {0, -1, 0},
            r(start = {0, R + R1, 0}),
            v(start = v0 + cross(omega0, {0, R1, 0})),
            q(start = {cos(pi/2), 0, 0, sin(pi/2)}),
            omega(start = omega0)) 
            annotation (Placement(transformation(extent={{-20,10},{0,30}})));
          RollerContactForces Contact3 
            annotation (Placement(transformation(extent={{-60,50},{-40,70}})));
          TwoPortsHeavyBody Roller3(
            Gravity = {0, -1, 0},
            r(start = {-R1, R, 0}),
            v(start = v0 + cross(omega0, {-R1, 0, 0})),
            q(start = {cos(3*pi/4), 0, 0, sin(3*pi/4)}),
            omega(start = omega0)) 
            annotation (Placement(transformation(extent={{-20,50},{0,70}})));
          FixedJoint Joint3(
            nA = {1, 0, 0},
            nB = {0, -1, 0},
            rA = {0, 0, 0},
            rB = {-R1, 0, 0}) 
            annotation (Placement(transformation(extent={{20,50},{40,70}})));
          FixedJoint Joint2(
            nA = {1, 0, 0},
            nB = {-1, 0, 0},
            rA = {0, 0, 0},
            rB = {0, R1, 0}) 
            annotation (Placement(transformation(extent={{20,10},{40,30}})));
          FixedJoint Joint1(
            nA = {1, 0, 0},
            nB = {0, 1, 0},
            rA = {0, 0, 0},
            rB = {R1, 0, 0}) 
            annotation (Placement(transformation(extent={{20,-30},{40,-10}})));
          FixedJoint Joint0(
            nA = {1, 0, 0},
            nB = {1, 0, 0},
            rA = {0, 0, 0},
            rB = {0, -R1, 0}) 
            annotation (Placement(transformation(extent={{20,-70},{40,-50}})));
          FourPortsHeavyBody Wheel(
            Gravity = {0, -1, 0},
            r(start = {0, R, 0}),
            v(start = v0),
            q(start = {1, 0, 0, 0}),
            omega(start = omega0)) 
            annotation (Placement(transformation(extent={{60,-10},{80,10}})));
        equation
          connect(Floor.OutPort, Contact0.InPortA)           annotation (Line(
              points={{-80,-8},{-80,-76},{-54,-76},{-54,-68}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact0.InPortB, Roller0.OutPort) 
            annotation (Line(
              points={{-46,-68},{-46,-76},{-10,-76},{-10,-68}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact0.OutPortB, Roller0.InPort) 
            annotation (Line(
              points={{-46,-52},{-46,-44},{-14,-44},{-14,-52.1}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact1.InPortA, Floor.OutPort) annotation (Line(
              points={{-54,-28},{-54,-36},{-80,-36},{-80,-8}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact1.InPortB, Roller1.OutPort) annotation (Line(
              points={{-46,-28},{-46,-36},{-10,-36},{-10,-28}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact1.OutPortB, Roller1.InPort) annotation (Line(
              points={{-46,-12},{-46,-4},{-14,-4},{-14,-12.1}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact2.InPortA, Floor.OutPort) annotation (Line(
              points={{-54,12},{-54,4},{-64,4},{-64,-16},{-80,-16},{-80,-8}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact2.InPortB, Roller2.OutPort) annotation (Line(
              points={{-46,12},{-46,4},{-10,4},{-10,12}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact3.InPortA, Floor.OutPort) annotation (Line(
              points={{-54,52},{-54,44},{-64,44},{-64,-16},{-80,-16},{-80,-8}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact2.OutPortB, Roller2.InPort) annotation (Line(
              points={{-46,28},{-46,36},{-14,36},{-14,27.9}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact3.InPortB, Roller3.OutPort) annotation (Line(
              points={{-46,52},{-46,44},{-10,44},{-10,52}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact3.OutPortB, Roller3.InPort) annotation (Line(
              points={{-46,68},{-46,76},{-14,76},{-14,67.9}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Roller3.InPort1, Joint3.OutPortA) annotation (Line(
              points={{-6,68},{-6,76},{26,76},{26,68}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Roller2.InPort1, Joint2.OutPortA) annotation (Line(
              points={{-6,28},{-6,36},{26,36},{26,28}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Roller1.InPort1, Joint1.OutPortA) annotation (Line(
              points={{-6,-12},{-6,-4},{26,-4},{26,-12}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Roller2.OutPort, Joint2.InPortA) annotation (Line(
              points={{-10,12},{-10,4},{26,4},{26,12}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Roller3.OutPort, Joint3.InPortA) annotation (Line(
              points={{-10,52},{-10,44},{26,44},{26,52}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Roller0.InPort1, Joint0.OutPortA) annotation (Line(
              points={{-6,-52},{-6,-44},{26,-44},{26,-52}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Roller0.OutPort, Joint0.InPortA) annotation (Line(
              points={{-10,-68},{-10,-76},{26,-76},{26,-68}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Roller1.OutPort, Joint1.InPortA) annotation (Line(
              points={{-10,-28},{-10,-36},{26,-36},{26,-28}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.InPort, Joint0.OutPortB) annotation (Line(
              points={{63.4,8},{64,8},{64,14},{56,14},{56,-44},{34,-44},{34,-52}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.InPort2, Joint2.OutPortB) annotation (Line(
              points={{72.2,8},{72.2,36},{34,36},{34,28}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.InPort1, Joint1.OutPortB) annotation (Line(
              points={{67.8,8},{68,8},{68,18},{52,18},{52,-4},{34,-4},{34,-12}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.InPort3, Joint3.OutPortB) annotation (Line(
              points={{76.6,8},{76,8},{76,76},{34,76},{34,68}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.OutPort, Joint3.InPortB) annotation (Line(
              points={{70,-8},{70,-16},{48,-16},{48,44},{34,44},{34,52}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.OutPort, Joint2.InPortB) annotation (Line(
              points={{70,-8},{70,-16},{48,-16},{48,4},{34,4},{34,12}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.OutPort, Joint1.InPortB) annotation (Line(
              points={{70,-8},{70,-36},{34,-36},{34,-28}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.OutPort, Joint0.InPortB) annotation (Line(
              points={{70,-8},{70,-76},{34,-76},{34,-68}},
              color={0,0,255},
              smooth=Smooth.None));
        end AutonomousOmniWheelTest;

        model OmniWheel
          parameter Integer n = 4 "Number of rollers";
          parameter Real alpha = Modelica.Constants.pi/n
            "Max angle of the half-sector";
          parameter Real R = 1 "Omni wheel outer radius";
          parameter Real R1 = R*cos(alpha) "Omni wheel inner radius";
          parameter Real[3] r0 = {0, R, 0};
          parameter Real[4] q0 = {1, 0, 0, 0};
          parameter Real[3] v0 = {1, 0, 0};
          parameter Real[3] omega0 = {0, 0, -1};
          parameter Real[3, 3] T0 = identity(3);
          parameter Real pi = Modelica.Constants.pi;
          parameter SI.Acceleration[3] Gravity = {0, -1, 0};
          TwoPortsHeavyBody Roller0(
            Gravity = Gravity,
            r(start = r0 + T0*{0, -R1, 0}),
            v(start = v0 + T0*cross(omega0, {0, -R1, 0})),
            q(start = QMult(q0, {1, 0, 0, 0})),
            omega(start = {0, omega0[2], omega0[3]} + {0, 0, 0})) 
            annotation (Placement(transformation(extent={{-30,-70},{-10,-50}})));
                                               //??
          RollerContactForces Contact0 
            annotation (Placement(transformation(extent={{-60,-70},{-40,-50}})));
          annotation (Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                    -100},{100,100}}),
                                 graphics),
            experiment(
              StopTime=40,
              NumberOfIntervals=50000,
              Tolerance=1e-008),
            experimentSetupOutput);
          RollerContactForces Contact1 
            annotation (Placement(transformation(extent={{-60,-30},{-40,-10}})));
          TwoPortsHeavyBody Roller1(
            Gravity = Gravity,
            r(start = r0 + T0*{R1, 0, 0}),
            v(start = v0 + T0*cross(omega0, {R1, 0, 0})),
            q(start = QMult(q0, {cos(pi/4), 0, 0, sin(pi/4)})),
            omega(start = {omega0[2], 0, omega0[3]} + {0, 0, 0})) 
            annotation (Placement(transformation(extent={{-30,-30},{-10,-10}})));
          RollerContactForces Contact2 
            annotation (Placement(transformation(extent={{-60,10},{-40,30}})));
          TwoPortsHeavyBody Roller2(
            Gravity = Gravity,
            r(start = r0 + T0*{0, R1, 0}),
            v(start = v0 + T0*cross(omega0, {0, R1, 0})),
            q(start = QMult(q0, {cos(pi/2), 0, 0, sin(pi/2)})),
            omega(start = {0, -omega0[2], omega0[3]} + {0, 0, 0})) 
            annotation (Placement(transformation(extent={{-30,10},{-10,30}})));
          RollerContactForces Contact3 
            annotation (Placement(transformation(extent={{-60,50},{-40,70}})));
          TwoPortsHeavyBody Roller3(
            Gravity = Gravity,
            r(start = r0 + T0*{-R1, 0, 0}),
            v(start = v0 + T0*cross(omega0, {-R1, 0, 0})),
            q(start = QMult(q0, {cos(3*pi/4), 0, 0, sin(3*pi/4)})),
            omega(start = {-omega0[2], 0, omega0[3]} + {0, 0, 0})) 
            annotation (Placement(transformation(extent={{-30,50},{-10,70}})));
        //  FixedJoint Joint3(
          Rigid Joint3(
            rA = {0, 0, 0},
            rB = {-R1, 0, 0}) 
            annotation (Placement(transformation(extent={{0,50},{20,70}})));
        //    nA = {1, 0, 0},
        //    nB = {0, -1, 0},
        //  FixedJoint Joint2(
          Rigid Joint2(
            rA = {0, 0, 0},
            rB = {0, R1, 0}) 
            annotation (Placement(transformation(extent={{0,10},{20,30}})));
        //    nA = {1, 0, 0},
        //    nB = {-1, 0, 0},
        //  FixedJoint Joint1(
          Rigid Joint1(
            rA = {0, 0, 0},
            rB = {R1, 0, 0}) 
            annotation (Placement(transformation(extent={{0,-30},{20,-10}})));
        //    nA = {1, 0, 0},
        //    nB = {0, 1, 0},
        //  FixedJoint Joint0(
          Rigid Joint0(
            rA = {0, 0, 0},
            rB = {0, -R1, 0}) 
            annotation (Placement(transformation(extent={{0,-70},{20,-50}})));
        //    nA = {1, 0, 0},
        //    nB = {1, 0, 0},
          FivePortsHeavyBody Wheel(
            Gravity = Gravity,
            r(start = r0),
            v(start = v0),
            q(start = q0),
            omega(start = omega0)) 
            annotation (Placement(transformation(extent={{40,-10},{60,10}})));
          KinematicPort InPortK 
            annotation (Placement(transformation(extent={{-50,-100},{-30,-80}})));
          WrenchPort InPortF 
            annotation (Placement(transformation(extent={{30,80},{50,100}})));
          KinematicPort OutPortK 
            annotation (Placement(transformation(extent={{30,-100},{50,-80}})));
        equation
          connect(Contact0.InPortB, Roller0.OutPort) 
            annotation (Line(
              points={{-46,-68},{-46,-76},{-20,-76},{-20,-68}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact0.OutPortB, Roller0.InPort) 
            annotation (Line(
              points={{-46,-52},{-46,-44},{-24,-44},{-24,-52.1}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact1.InPortB, Roller1.OutPort) annotation (Line(
              points={{-46,-28},{-46,-36},{-20,-36},{-20,-28}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact1.OutPortB, Roller1.InPort) annotation (Line(
              points={{-46,-12},{-46,-4},{-24,-4},{-24,-12.1}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact2.InPortB, Roller2.OutPort) annotation (Line(
              points={{-46,12},{-46,4},{-20,4},{-20,12}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact2.OutPortB, Roller2.InPort) annotation (Line(
              points={{-46,28},{-46,36},{-24,36},{-24,27.9}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact3.InPortB, Roller3.OutPort) annotation (Line(
              points={{-46,52},{-46,44},{-20,44},{-20,52}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact3.OutPortB, Roller3.InPort) annotation (Line(
              points={{-46,68},{-46,76},{-24,76},{-24,67.9}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Roller3.InPort1, Joint3.OutPortA) annotation (Line(
              points={{-16,68},{-16,76},{6,76},{6,68}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Roller2.InPort1, Joint2.OutPortA) annotation (Line(
              points={{-16,28},{-16,36},{6,36},{6,28}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Roller1.InPort1, Joint1.OutPortA) annotation (Line(
              points={{-16,-12},{-16,-4},{6,-4},{6,-12}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Roller2.OutPort, Joint2.InPortA) annotation (Line(
              points={{-20,12},{-20,4},{6,4},{6,12}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Roller3.OutPort, Joint3.InPortA) annotation (Line(
              points={{-20,52},{-20,44},{6,44},{6,52}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Roller0.InPort1, Joint0.OutPortA) annotation (Line(
              points={{-16,-52},{-16,-44},{6,-44},{6,-52}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Roller0.OutPort, Joint0.InPortA) annotation (Line(
              points={{-20,-68},{-20,-76},{6,-76},{6,-68}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Roller1.OutPort, Joint1.InPortA) annotation (Line(
              points={{-20,-28},{-20,-36},{6,-36},{6,-28}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.InPort, Joint0.OutPortB) annotation (Line(
              points={{43.4,8},{44,8},{44,14},{36,14},{36,-44},{14,-44},{14,-52}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.InPort2, Joint2.OutPortB) annotation (Line(
              points={{52.2,8},{52.2,36},{14,36},{14,28}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.InPort1, Joint1.OutPortB) annotation (Line(
              points={{47.8,8},{48,8},{48,18},{32,18},{32,-4},{14,-4},{14,-12}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.InPort3, Joint3.OutPortB) annotation (Line(
              points={{56.6,8},{56,8},{56,76},{14,76},{14,68}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.OutPort, Joint3.InPortB) annotation (Line(
              points={{50,-8},{50,-16},{28,-16},{28,44},{14,44},{14,52}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.OutPort, Joint2.InPortB) annotation (Line(
              points={{50,-8},{50,-16},{28,-16},{28,4},{14,4},{14,12}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.OutPort, Joint1.InPortB) annotation (Line(
              points={{50,-8},{50,-36},{14,-36},{14,-28}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.OutPort, Joint0.InPortB) annotation (Line(
              points={{50,-8},{50,-76},{14,-76},{14,-68}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.InPort4, InPortF) annotation (Line(
              points={{58,0},{70,0},{70,90},{40,90}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.OutPort, OutPortK) annotation (Line(
              points={{50,-8},{50,-76},{40,-76},{40,-90}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact1.InPortA, InPortK) annotation (Line(
              points={{-54,-28},{-54,-36},{-70,-36},{-70,-90},{-40,-90}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact0.InPortA, InPortK) annotation (Line(
              points={{-54,-68},{-56,-68},{-56,-90},{-40,-90}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact2.InPortA, InPortK) annotation (Line(
              points={{-54,12},{-54,4},{-70,4},{-70,-90},{-40,-90}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact3.InPortA, InPortK) annotation (Line(
              points={{-54,52},{-54,44},{-70,44},{-70,-90},{-40,-90}},
              color={0,0,255},
              smooth=Smooth.None));
        end OmniWheel;

        model AutonomousPatchContactOmniWheelSetTest
          parameter Integer n = 4 "Number of rollers";
          parameter Real alpha = Modelica.Constants.pi/n
            "Max angle of the half-sector";
          parameter Real R = 1 "Omni wheel outer radius";
          parameter Real d = 2 "vehicle size";
          parameter Real R1 = R*cos(alpha) "Omni wheel inner radius";
          parameter Real om0 = 0.1;
          parameter Real[3] r0 = {0, R, 0};
          parameter Real[4] q0 = {1, 0, 0, 0};
          parameter Real[3] v0 = {0, 0, 0};
          parameter Real[3] omega0 = {0, om0, 0};
          parameter Real pi = Modelica.Constants.pi;
        //  Real[3] d;
          Base Floor 
                    annotation (Placement(transformation(extent={{-90,-10},{-70,10}})));
          annotation (Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                    -100},{100,100}}),
                                 graphics),
            experiment(
              StopTime=10,
              NumberOfIntervals=50000,
              Tolerance=1e-009,
              Algorithm="Dassl"),
            experimentSetupOutput);
          FixedJoint Joint1(
            nA = {0, 0, 1},
            nB = {1, 0, 0},
            rA = {0, 0, 0},
            rB = {d, 0, 0}) 
            annotation (Placement(transformation(extent={{20,40},{40,60}})));
          FixedJoint Joint2(
            nA = {0, 0, 1},
            nB = {-cos(pi/3), 0, -cos(pi/6)},
            rA = {0, 0, 0},
            rB = {-d*cos(pi/3), 0, -d*cos(pi/6)}) 
            annotation (Placement(transformation(extent={{20,-10},{40,10}})));
          ThreePortsHeavyBody Platform(
            Gravity = {0, -1, 0},
            r(start = r0),
            v(start = v0),
            q(start = q0),
            omega(start = omega0)) 
            annotation (Placement(transformation(extent={{70,-10},{90,10}})));
          OmniWheel Wheel1(
              Gravity = {0, -1, 0},
              T0 = [0, 0, -1; 0, 1, 0; 1, 0, 0],
              r0 = r0 + {d, 0, 0},
              v0 = v0 + cross(omega0, {d, 0, 0}),
              q0 = QMult(q0, {cos(pi/4),0,sin(pi/4),0}),
              omega0 = {0, om0, -d*om0/R}) 
            annotation (Placement(transformation(extent={{-40,40},{-20,60}})));
          OmniWheel Wheel2(
              Gravity = {0, -1, 0},
              T0 = [-cos(pi/6), 0, cos(pi/3); 0, 1, 0; -cos(pi/3), 0, -cos(pi/6)],
              r0 = r0 + {-d*cos(pi/3), 0, -d*cos(pi/6)},
              v0 = v0 + cross(omega0, {-d*cos(pi/3), 0, -d*cos(pi/6)}),
              q0 = QMult(q0, {cos(7*pi/12), 0, sin(7*pi/12),0}),
              omega0 = {0, om0, -d*om0/R}) 
            annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
          OmniWheel Wheel3(
              Gravity = {0, -1, 0},
              T0 = [cos(pi/6), 0, cos(pi/3); 0, 1, 0; -cos(pi/3), 0, cos(pi/6)],
              r0 = r0 + {-d*cos(pi/3), 0, d*cos(pi/6)},
              v0 = v0 + cross(omega0, {-d*cos(pi/3), 0, d*cos(pi/6)}),
              q0 = QMult(q0, {cos(-pi/12), 0, sin(-pi/12), 0}),
              omega0 = {0, om0, -d*om0/R}) 
            annotation (Placement(transformation(extent={{-40,-60},{-20,-40}})));
          FixedJoint Joint3(
            nA = {0, 0, 1},
            nB = {-cos(pi/3), 0, cos(pi/6)},
            rA = {0, 0, 0},
            rB = {-d*cos(pi/3), 0, d*cos(pi/6)}) 
            annotation (Placement(transformation(extent={{20,-60},{40,-40}})));
        equation
        //  d = cross({0, 1, 0}, Wheel.OutPort.T*{0, 0, 1});
        //  Wheel.OutPort.epsilon*d/sqrt(d*d) = 0;
          connect(Floor.OutPort, Wheel2.InPortK) annotation (Line(
              points={{-80,-8},{-80,-20},{-34,-20},{-34,-9}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel1.OutPortK,Joint1. InPortA) annotation (Line(
              points={{-26,41},{-26,30},{26,30},{26,42}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel2.OutPortK, Joint2.InPortA) annotation (Line(
              points={{-26,-9},{-26,-20},{26,-20},{26,-8}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel1.InPortF,Joint1. OutPortA) annotation (Line(
              points={{-26,59},{-26,70},{26,70},{26,58}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel2.InPortF, Joint2.OutPortA) annotation (Line(
              points={{-26,9},{-26,20},{26,20},{26,8}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Platform.OutPort,Joint1. InPortB) annotation (Line(
              points={{80,-8},{80,-20},{60,-20},{60,30},{34,30},{34,42}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Platform.OutPort, Joint2.InPortB) annotation (Line(
              points={{80,-8},{80,-20},{34,-20},{34,-8}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Joint2.OutPortB, Platform.InPort1) annotation (Line(
              points={{34,8},{34,20},{80,20},{80,8}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Joint1.OutPortB, Platform.InPort) annotation (Line(
              points={{34,58},{34,70},{74,70},{74,8}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Floor.OutPort, Wheel3.InPortK) annotation (Line(
              points={{-80,-8},{-80,-70},{-34,-70},{-34,-59}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel1.InPortK, Floor.OutPort) annotation (Line(
              points={{-34,41},{-34,30},{-50,30},{-50,-20},{-80,-20},{-80,-8}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel3.OutPortK, Joint3.InPortA) annotation (Line(
              points={{-26,-59},{-26,-70},{26,-70},{26,-58}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Joint3.InPortB, Platform.OutPort) annotation (Line(
              points={{34,-58},{34,-70},{80,-70},{80,-8}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel3.InPortF, Joint3.OutPortA) annotation (Line(
              points={{-26,-41},{-26,-30},{26,-30},{26,-42}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Joint3.OutPortB, Platform.InPort2) annotation (Line(
              points={{34,-42},{34,-30},{66,-30},{66,30},{86,30},{86,8}},
              color={0,0,255},
              smooth=Smooth.None));
        end AutonomousPatchContactOmniWheelSetTest;

        model OmniWheel1
          parameter Integer n = 4 "Number of rollers";
          parameter Real alpha = Modelica.Constants.pi/n
            "Max angle of the half-sector";
          parameter Real R = 1 "Omni wheel outer radius";
          parameter Real R1 = R*cos(alpha) "Omni wheel inner radius";
          parameter Real[3] r0 = {0, R, 0};
          parameter Real[4] q0 = {1, 0, 0, 0};
          parameter Real[3] v0 = {1, 0, 0};
          parameter Real[3] omega0 = {0, 0, -1};
          parameter Real[3, 3] T0 = identity(3);
          parameter Real pi = Modelica.Constants.pi;
          parameter SI.Acceleration[3] Gravity = {0, -1, 0};
          TwoPortsHeavyBody Roller0(
            Gravity = Gravity,
            r(start = r0 + T0*{0, -R1, 0}),
            v(start = v0 + T0*cross(omega0, {0, -R1, 0})),
            q(start = QMult(q0, {1, 0, 0, 0})),
            omega(start = {0, omega0[2], omega0[3]} + {0, 0, 0})) 
            annotation (Placement(transformation(extent={{-30,-70},{-10,-50}})));
                                               //??
          RollerContactForces Contact0 
            annotation (Placement(transformation(extent={{-60,-70},{-40,-50}})));
          annotation (Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                    -100},{100,100}}),
                                 graphics),
            experiment(
              StopTime=40,
              NumberOfIntervals=50000,
              Tolerance=1e-008),
            experimentSetupOutput);
          RollerContactForces Contact1 
            annotation (Placement(transformation(extent={{-60,-30},{-40,-10}})));
          TwoPortsHeavyBody Roller1(
            Gravity = Gravity,
            r(start = r0 + T0*{R1, 0, 0}),
            v(start = v0 + T0*cross(omega0, {R1, 0, 0})),
            q(start = QMult(q0, {cos(pi/4), 0, 0, sin(pi/4)})),
            omega(start = {omega0[2], 0, omega0[3]} + {0, 0, 0})) 
            annotation (Placement(transformation(extent={{-30,-30},{-10,-10}})));
          RollerContactForces Contact2 
            annotation (Placement(transformation(extent={{-60,10},{-40,30}})));
          TwoPortsHeavyBody Roller2(
            Gravity = Gravity,
            r(start = r0 + T0*{0, R1, 0}),
            v(start = v0 + T0*cross(omega0, {0, R1, 0})),
            q(start = QMult(q0, {cos(pi/2), 0, 0, sin(pi/2)})),
            omega(start = {0, -omega0[2], omega0[3]} + {0, 0, 0})) 
            annotation (Placement(transformation(extent={{-30,10},{-10,30}})));
          RollerContactForces Contact3 
            annotation (Placement(transformation(extent={{-60,50},{-40,70}})));
          TwoPortsHeavyBody Roller3(
            Gravity = Gravity,
            r(start = r0 + T0*{-R1, 0, 0}),
            v(start = v0 + T0*cross(omega0, {-R1, 0, 0})),
            q(start = QMult(q0, {cos(3*pi/4), 0, 0, sin(3*pi/4)})),
            omega(start = {-omega0[2], 0, omega0[3]} + {0, 0, 0})) 
            annotation (Placement(transformation(extent={{-30,50},{-10,70}})));
        //  FixedJoint Joint3(
          Rigid Joint3(
            rA = {0, 0, 0},
            rB = {-R1, 0, 0}) 
            annotation (Placement(transformation(extent={{0,50},{20,70}})));
        //    nA = {1, 0, 0},
        //    nB = {0, -1, 0},
        //  FixedJoint Joint2(
          Rigid Joint2(
            rA = {0, 0, 0},
            rB = {0, R1, 0}) 
            annotation (Placement(transformation(extent={{0,10},{20,30}})));
        //    nA = {1, 0, 0},
        //    nB = {-1, 0, 0},
        //  FixedJoint Joint1(
          Rigid Joint1(
            rA = {0, 0, 0},
            rB = {R1, 0, 0}) 
            annotation (Placement(transformation(extent={{0,-30},{20,-10}})));
        //    nA = {1, 0, 0},
        //    nB = {0, 1, 0},
        //  FixedJoint Joint0(
          Rigid Joint0(
            rA = {0, 0, 0},
            rB = {0, -R1, 0}) 
            annotation (Placement(transformation(extent={{0,-70},{20,-50}})));
        //    nA = {1, 0, 0},
        //    nB = {1, 0, 0},
          FivePortsHeavyBody Wheel(
            Gravity = Gravity,
            r(start = r0),
            v(start = v0),
            q(start = q0),
            omega(start = omega0)) 
            annotation (Placement(transformation(extent={{40,-10},{60,10}})));
          KinematicPort InPortK 
            annotation (Placement(transformation(extent={{-50,-100},{-30,-80}})));
          WrenchPort InPortF 
            annotation (Placement(transformation(extent={{30,80},{50,100}})));
          KinematicPort OutPortK 
            annotation (Placement(transformation(extent={{30,-100},{50,-80}})));
        equation
          connect(Contact0.InPortB, Roller0.OutPort) 
            annotation (Line(
              points={{-46,-68},{-46,-76},{-20,-76},{-20,-68}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact0.OutPortB, Roller0.InPort) 
            annotation (Line(
              points={{-46,-52},{-46,-44},{-24,-44},{-24,-52.1}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact1.InPortB, Roller1.OutPort) annotation (Line(
              points={{-46,-28},{-46,-36},{-20,-36},{-20,-28}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact1.OutPortB, Roller1.InPort) annotation (Line(
              points={{-46,-12},{-46,-4},{-24,-4},{-24,-12.1}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact2.InPortB, Roller2.OutPort) annotation (Line(
              points={{-46,12},{-46,4},{-20,4},{-20,12}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact2.OutPortB, Roller2.InPort) annotation (Line(
              points={{-46,28},{-46,36},{-24,36},{-24,27.9}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact3.InPortB, Roller3.OutPort) annotation (Line(
              points={{-46,52},{-46,44},{-20,44},{-20,52}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact3.OutPortB, Roller3.InPort) annotation (Line(
              points={{-46,68},{-46,76},{-24,76},{-24,67.9}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Roller3.InPort1, Joint3.OutPortA) annotation (Line(
              points={{-16,68},{-16,76},{6,76},{6,68}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Roller2.InPort1, Joint2.OutPortA) annotation (Line(
              points={{-16,28},{-16,36},{6,36},{6,28}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Roller1.InPort1, Joint1.OutPortA) annotation (Line(
              points={{-16,-12},{-16,-4},{6,-4},{6,-12}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Roller2.OutPort, Joint2.InPortA) annotation (Line(
              points={{-20,12},{-20,4},{6,4},{6,12}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Roller3.OutPort, Joint3.InPortA) annotation (Line(
              points={{-20,52},{-20,44},{6,44},{6,52}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Roller0.InPort1, Joint0.OutPortA) annotation (Line(
              points={{-16,-52},{-16,-44},{6,-44},{6,-52}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Roller0.OutPort, Joint0.InPortA) annotation (Line(
              points={{-20,-68},{-20,-76},{6,-76},{6,-68}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Roller1.OutPort, Joint1.InPortA) annotation (Line(
              points={{-20,-28},{-20,-36},{6,-36},{6,-28}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.InPort, Joint0.OutPortB) annotation (Line(
              points={{43.4,8},{44,8},{44,14},{36,14},{36,-44},{14,-44},{14,-52}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.InPort2, Joint2.OutPortB) annotation (Line(
              points={{52.2,8},{52.2,36},{14,36},{14,28}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.InPort1, Joint1.OutPortB) annotation (Line(
              points={{47.8,8},{48,8},{48,18},{32,18},{32,-4},{14,-4},{14,-12}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.InPort3, Joint3.OutPortB) annotation (Line(
              points={{56.6,8},{56,8},{56,76},{14,76},{14,68}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.OutPort, Joint3.InPortB) annotation (Line(
              points={{50,-8},{50,-16},{28,-16},{28,44},{14,44},{14,52}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.OutPort, Joint2.InPortB) annotation (Line(
              points={{50,-8},{50,-16},{28,-16},{28,4},{14,4},{14,12}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.OutPort, Joint1.InPortB) annotation (Line(
              points={{50,-8},{50,-36},{14,-36},{14,-28}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.OutPort, Joint0.InPortB) annotation (Line(
              points={{50,-8},{50,-76},{14,-76},{14,-68}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.InPort4, InPortF) annotation (Line(
              points={{58,0},{70,0},{70,90},{40,90}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.OutPort, OutPortK) annotation (Line(
              points={{50,-8},{50,-76},{40,-76},{40,-90}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact1.InPortA, InPortK) annotation (Line(
              points={{-54,-28},{-54,-36},{-70,-36},{-70,-90},{-40,-90}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact0.InPortA, InPortK) annotation (Line(
              points={{-54,-68},{-56,-68},{-56,-90},{-40,-90}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact2.InPortA, InPortK) annotation (Line(
              points={{-54,12},{-54,4},{-70,4},{-70,-90},{-40,-90}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact3.InPortA, InPortK) annotation (Line(
              points={{-54,52},{-54,44},{-70,44},{-70,-90},{-40,-90}},
              color={0,0,255},
              smooth=Smooth.None));
        end OmniWheel1;
      end PatchContact;

      package PointContact
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
        //  SI.Velocity mu;
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
          extends Constraint;
          parameter Real[3] nA;
          parameter Real[3] nB;
          parameter SI.Position[3] rA = zeros(3)
            "Constraint position in body A";
          parameter SI.Position[3] rB = zeros(3)
            "Constraint position in body B";
          SI.Position[3] RA;
          SI.Position[3] RB;
          SI.Velocity[3] vA;
          SI.Velocity[3] vB;
          SI.AngularAcceleration lambda;
        //  SI.AngularVelocity mu;
          SI.AngularVelocity[3] omegar "Relative angular velocity";
          SI.AngularAcceleration[3] epsilonr "Relative angular acceleration";
          Real nAi[3] "Unit vector of joint axis w. r. t. inertial frame";
          SI.Torque M "Torque about joint axis";
        equation
          RA = InPortA.r + InPortA.T*rA;
          RB = InPortB.r + InPortB.T*rB;
          vA = InPortA.v + cross(InPortA.omega, InPortA.T*rA);
          vB = InPortB.v + cross(InPortB.omega, InPortB.T*rB);
          vA = vB;

          nAi = InPortA.T*nA;

        //  InPortA.epsilon = InPortB.epsilon;
        //  cross(InPortA.T*nA, InPortB.T*nB) = zeros(3);
        //  InPortB.T*nB = mu*(InPortA.T*nA);
        //  InPortB.omega - InPortA.omega = lambda*InPortA.T*nA;

          omegar = InPortB.omega - InPortA.omega;
        //  omegar = lambda*nAi;
          epsilonr = InPortB.epsilon - InPortA.epsilon - cross(InPortA.omega, omegar);
          epsilonr = lambda*nAi;

          M = OutPortA.M*nAi;
          M = 0;

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
        end FixedJoint;

        partial model RollerPointContactVelocities
          extends RollerContactTracking;
          Real[3] vA;
          Real[3] vB;
          Real[3] relv;
          Real relvn;
          Real vAn;
          Real vBn;
          Real[3] vAt;
          Real[3] vBt;
          Real[3] relvt;
          Real relvtsqrt;
        equation
          vA = InPortA.v + cross(InPortA.omega, rA - InPortA.r);
          vB = InPortB.v + cross(InPortB.omega, rB - InPortB.r);
          relv = vB - vA;
          relvn = relv*nA;
          vAn = vA*nA;
          vBn = vB*nA;
          vAt = vA - vAn*nA;
          vBt = vB - vBn*nA;
          relvt = vBt - vAt;
          relvtsqrt = sqrt(relvt*relvt);

          OutPortA.P = rA;
          OutPortB.P = rB;
        end RollerPointContactVelocities;

        partial model RollerPointContactVelocitiesGeneral
          extends RollerContactTrackingGeneral;
          Real[3] vA;
          Real[3] vB;
          Real[3] relv;
          Real relvn;
          Real vAn;
          Real vBn;
          Real[3] vAt;
          Real[3] vBt;
          Real[3] relvt;
          Real relvtsqrt;
        equation
          vA = InPortA.v + cross(InPortA.omega, rA - InPortA.r);
          vB = InPortB.v + cross(InPortB.omega, rB - InPortB.r);
          relv = vB - vA;
          relvn = relv*nA;
          vAn = vA*nA;
          vBn = vB*nA;
          vAt = vA - vAn*nA;
          vBt = vB - vBn*nA;
          relvt = vBt - vAt;
          relvtsqrt = sqrt(relvt*relvt);

          OutPortA.P = rA;
          OutPortB.P = rB;
        end RollerPointContactVelocitiesGeneral;

        partial model RollerPointContactVelocitiesGeneralStep
          extends RollerContactTrackingGeneralStep;
          Real[3] vA;
          Real[3] vB;
          Real[3] relv;
          Real relvn;
          Real vAn;
          Real vBn;
          Real[3] vAt;
          Real[3] vBt;
          Real[3] relvt;
          Real relvtsqrt;
        equation
          vA = InPortA.v + cross(InPortA.omega, rA - InPortA.r);
          vB = InPortB.v + cross(InPortB.omega, rB - InPortB.r);
          relv = vB - vA;
          relvn = relv*nA;
          vAn = vA*nA;
          vBn = vB*nA;
          vAt = vA - vAn*nA;
          vBt = vB - vBn*nA;
          relvt = vBt - vAt;
          relvtsqrt = sqrt(relvt*relvt);

          OutPortA.P = rA;
          OutPortB.P = rB;
        end RollerPointContactVelocitiesGeneralStep;

        model RollerPointContactForces
          import ThreeD_MBS_Dynamics;
          extends RollerPointContactVelocities;
          parameter Real delta = 10^(-6);
          parameter Real fric = 0.1;
          Real mu(start = 0);
          Real[3] Forcet(start = zeros(3));
          Real Drelvn;
          Real Forcen;
          Real ForceTsqrt;
          Real w;

          Real isInContact;
        equation
          w = (InPortB.T*i)*nA;

          if noEvent(abs((InPortB.T*i)*nA) < cos_of_max and h < R) then //??
        //    relvn = 0;
            Drelvn = 0;
        //    rB[2] = 0;

        //    Physical:
        //    if (noEvent(abs((InPortB.T*i)*nA) < cos(Modelica.Constants.pi/2 - alpha + 7 * Modelica.Constants.pi/180))) then
              Forcet = -fric*relvt*(if noEvent(relvtsqrt <= delta) then 1/delta else 1/relvtsqrt)*Forcen + mu*nA;
        //    else
        //      Forcet = zeros(3);
        //    end if;

        //    Shaman:
        //    Forcet = (-fric*relvt*(if noEvent(relvtsqrt <= delta) then 1/delta else 1/relvtsqrt)*Forcen + mu*nA)/(if noEvent(ForceTsqrt < 10^(-3)) then 1 else 10000000);
            isInContact = 1;
          else
            Forcen = 0;
            Forcet = zeros(3);
            isInContact = 0;
          end if;
          Drelvn = der(relvn);
          Forcen = OutPortB.F*nA;
          ForceTsqrt = sqrt(Forcet*Forcet);

          OutPortB.F = Forcet + Forcen*nA;
          OutPortB.M = zeros(3);
        end RollerPointContactForces;

        model RollerPointContactForcesGeneral
          import ThreeD_MBS_Dynamics;
          extends RollerPointContactVelocitiesGeneral;
          parameter Real delta = 10^(-6);
          parameter Real fric = 0.1;
          Real mu;
        //  Real mu(stateSelect = StateSelect.prefer);
          Real[3] Forcet;
        //  Real[3] Forcet(start = zeros(3));
          Real Drelvn;
          Real Forcen;
          Real isInContact;
        /*
initial equation 
  if noEvent(abs(cosBtwAxisAndVert) < cos_of_max and h < R) then
    Forcen = 1;
  else
    mu = 0;
  end if;
*/
        equation
          if noEvent(abs(cosBtwAxisAndVert) < cos_of_max and InPortB.r[2] < R) then
            isInContact = 1;
            // Signorini:
        //    Drelvn = -200*relvn - 1000*h;
            Drelvn = 0;
        //    Physical:
        //    {Forcet[1], Forcet[3]} = -fric*{relvt[1], relvt[3]}*(if noEvent(relvtsqrt <= delta) then 1/delta else 1/relvtsqrt)*Forcen;
            Forcet = -fric*relvt*(if noEvent(relvtsqrt <= delta) then 1/delta else 1/relvtsqrt)*Forcen + mu*nA;
        //    Forcet[2] = 0;
          else
            isInContact = 0;
            // Signorini:
            Forcen = 0;
            Forcet = zeros(3);
          end if;

          Drelvn = der(relvn);
          Forcen = OutPortB.F*nA;

          OutPortB.F = Forcet + Forcen*nA;
          OutPortB.M = zeros(3);

          annotation (experiment(
              StopTime=10,
              NumberOfIntervals=50000,
              Tolerance=1e-009), experimentSetupOutput);
        end RollerPointContactForcesGeneral;

        model RollerPointContactForcesGeneralStep
          import ThreeD_MBS_Dynamics;
          extends RollerPointContactVelocitiesGeneralStep;
          parameter Real delta = 10^(-6);
          parameter Real fric = 0.1;
          Real mu;
        //  Real mu(stateSelect = StateSelect.prefer);
          Real[3] Forcet;
        //  Real[3] Forcet(start = zeros(3));
          Real Drelvn;
          Real Forcen;
          Real isInContact;
        /*
initial equation 
  if noEvent(abs(cosBtwAxisAndVert) < cos_of_max and h < R) then
    Forcen = 1;
  else
    mu = 0;
  end if;
*/
        equation
          if noEvent(abs(cosBtwAxisAndVert) < cos_of_max and InPortB.r[2] < R) then
            isInContact = 1;
            // Signorini:
        //    Drelvn = -200*relvn - 1000*h;
            Drelvn = 0;
        //    Physical:
        //    {Forcet[1], Forcet[3]} = -fric*{relvt[1], relvt[3]}*(if noEvent(relvtsqrt <= delta) then 1/delta else 1/relvtsqrt)*Forcen;
            Forcet = -fric*relvt*(if noEvent(relvtsqrt <= delta) then 1/delta else 1/relvtsqrt)*Forcen + mu*nA;
        //    Forcet[2] = 0;
          else
            isInContact = 0;
            // Signorini:
            Forcen = 0;
            Forcet = zeros(3);
          end if;

          Drelvn = der(relvn);
          Forcen = OutPortB.F*nA;

          OutPortB.F = Forcet + Forcen*nA;
          OutPortB.M = zeros(3);

          annotation (experiment(
              StopTime=10,
              NumberOfIntervals=50000,
              Tolerance=1e-009), experimentSetupOutput);
        end RollerPointContactForcesGeneralStep;

        model AutonomousPointContactOmniWheelTest
          parameter Integer n = 4 "Number of rollers";
          parameter Real alpha = Modelica.Constants.pi/n
            "Max angle of the half-sector";
          parameter Real R = 1 "Omni wheel outer radius";
          parameter Real R1 = R*cos(alpha) "Omni wheel inner radius";
          parameter Real[3] v0 = {1, 0, 0};
          parameter Real[3] omega0 = {0, 1, -1};
          parameter Real pi = Modelica.Constants.pi;
        //  Real[3] d;
          Base Floor 
                    annotation (Placement(transformation(extent={{-90,-10},{-70,10}})));
          TwoPortsHeavyBody Roller0(
            Gravity = {0, -1, 0},
            r(start = {0, R - R1, 0}),
            v(start = v0 + cross(omega0, {0, -R1, 0})),
            q(start = {1, 0, 0, 0}),
            omega(start = omega0)) 
            annotation (Placement(transformation(extent={{-20,-70},{0,-50}})));
          RollerPointContactForces Contact0(relvn(start = 0)) 
            annotation (Placement(transformation(extent={{-60,-70},{-40,-50}})));
          annotation (Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                    -100},{100,100}}),
                                 graphics),
            experiment(
              StopTime=10,
              NumberOfIntervals=50000,
              Tolerance=1e-009,
              Algorithm="Dassl"),
            experimentSetupOutput);
          RollerPointContactForces Contact1 
            annotation (Placement(transformation(extent={{-60,-30},{-40,-10}})));
          TwoPortsHeavyBody Roller1(
            Gravity = {0, -1, 0},
            r(start = {R1, R, 0}),
            v(start = v0 + cross(omega0, {R1, 0, 0})),
            q(start = {cos(pi/4), 0, 0, sin(pi/4)}),
            omega(start = {omega0[2], 0, omega0[3]} + {0, 0, 0})) 
            annotation (Placement(transformation(extent={{-20,-30},{0,-10}})));
          RollerPointContactForces Contact2 
            annotation (Placement(transformation(extent={{-60,10},{-40,30}})));
          TwoPortsHeavyBody Roller2(
            Gravity = {0, -1, 0},
            r(start = {0, R + R1, 0}),
            v(start = v0 + cross(omega0, {0, R1, 0})),
            q(start = {cos(pi/2), 0, 0, sin(pi/2)}),
            omega(start = {0, -omega0[2], omega0[3]} + {0, 0, 0})) 
            annotation (Placement(transformation(extent={{-20,10},{0,30}})));
          RollerPointContactForces Contact3 
            annotation (Placement(transformation(extent={{-60,50},{-40,70}})));
          TwoPortsHeavyBody Roller3(
            Gravity = {0, -1, 0},
            r(start = {-R1, R, 0}),
            v(start = v0 + cross(omega0, {-R1, 0, 0})),
            q(start = {cos(3*pi/4), 0, 0, sin(3*pi/4)}),
            omega(start = {-omega0[2], 0, omega0[3]} + {0, 0, 0})) 
            annotation (Placement(transformation(extent={{-20,50},{0,70}})));
          FixedJoint Joint3(
            nA = {1, 0, 0},
            nB = {0, -1, 0},
            rA = {0, 0, 0},
            rB = {-R1, 0, 0}) 
            annotation (Placement(transformation(extent={{20,50},{40,70}})));
          FixedJoint Joint2(
            nA = {1, 0, 0},
            nB = {-1, 0, 0},
            rA = {0, 0, 0},
            rB = {0, R1, 0}) 
            annotation (Placement(transformation(extent={{20,10},{40,30}})));
          FixedJoint Joint1(
            nA = {1, 0, 0},
            nB = {0, 1, 0},
            rA = {0, 0, 0},
            rB = {R1, 0, 0}) 
            annotation (Placement(transformation(extent={{20,-30},{40,-10}})));
          FixedJoint Joint0(
            nA = {1, 0, 0},
            nB = {1, 0, 0},
            rA = {0, 0, 0},
            rB = {0, -R1, 0}) 
            annotation (Placement(transformation(extent={{20,-70},{40,-50}})));
          FourPortsHeavyBody Wheel(
            Gravity = {0, -1, 0},
            r(start = {0, R, 0}),
            v(start = v0),
            q(start = {1, 0, 0, 0}),
            omega(start = omega0)) 
            annotation (Placement(transformation(extent={{60,-10},{80,10}})));
        equation
        //  d = cross({0, 1, 0}, Wheel.OutPort.T*{0, 0, 1});
        //  Wheel.OutPort.epsilon*d/sqrt(d*d) = 0;
          connect(Floor.OutPort, Contact0.InPortA)           annotation (Line(
              points={{-80,-8},{-80,-76},{-54,-76},{-54,-68}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact0.InPortB, Roller0.OutPort) 
            annotation (Line(
              points={{-46,-68},{-46,-76},{-10,-76},{-10,-68}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact0.OutPortB, Roller0.InPort) 
            annotation (Line(
              points={{-46,-52},{-46,-44},{-14,-44},{-14,-52.1}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact1.InPortA, Floor.OutPort) annotation (Line(
              points={{-54,-28},{-54,-36},{-80,-36},{-80,-8}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact1.InPortB, Roller1.OutPort) annotation (Line(
              points={{-46,-28},{-46,-36},{-10,-36},{-10,-28}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact1.OutPortB, Roller1.InPort) annotation (Line(
              points={{-46,-12},{-46,-4},{-14,-4},{-14,-12.1}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact2.InPortA, Floor.OutPort) annotation (Line(
              points={{-54,12},{-54,4},{-64,4},{-64,-16},{-80,-16},{-80,-8}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact2.InPortB, Roller2.OutPort) annotation (Line(
              points={{-46,12},{-46,4},{-10,4},{-10,12}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact3.InPortA, Floor.OutPort) annotation (Line(
              points={{-54,52},{-54,44},{-64,44},{-64,-16},{-80,-16},{-80,-8}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact3.InPortB, Roller3.OutPort) annotation (Line(
              points={{-46,52},{-46,44},{-10,44},{-10,52}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact3.OutPortB, Roller3.InPort) annotation (Line(
              points={{-46,68},{-46,76},{-14,76},{-14,67.9}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Roller3.InPort1, Joint3.OutPortA) annotation (Line(
              points={{-6,68},{-6,76},{26,76},{26,68}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Roller2.InPort1, Joint2.OutPortA) annotation (Line(
              points={{-6,28},{-6,36},{26,36},{26,28}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Roller1.InPort1, Joint1.OutPortA) annotation (Line(
              points={{-6,-12},{-6,-4},{26,-4},{26,-12}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Roller2.OutPort, Joint2.InPortA) annotation (Line(
              points={{-10,12},{-10,4},{26,4},{26,12}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Roller3.OutPort, Joint3.InPortA) annotation (Line(
              points={{-10,52},{-10,44},{26,44},{26,52}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Roller0.InPort1, Joint0.OutPortA) annotation (Line(
              points={{-6,-52},{-6,-44},{26,-44},{26,-52}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Roller0.OutPort, Joint0.InPortA) annotation (Line(
              points={{-10,-68},{-10,-76},{26,-76},{26,-68}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Roller1.OutPort, Joint1.InPortA) annotation (Line(
              points={{-10,-28},{-10,-36},{26,-36},{26,-28}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.InPort, Joint0.OutPortB) annotation (Line(
              points={{63.4,8},{64,8},{64,14},{56,14},{56,-44},{34,-44},{34,-52}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.InPort2, Joint2.OutPortB) annotation (Line(
              points={{72.2,8},{72.2,36},{34,36},{34,28}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.InPort1, Joint1.OutPortB) annotation (Line(
              points={{67.8,8},{68,8},{68,18},{52,18},{52,-4},{34,-4},{34,-12}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.InPort3, Joint3.OutPortB) annotation (Line(
              points={{76.6,8},{76,8},{76,76},{34,76},{34,68}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.OutPort, Joint3.InPortB) annotation (Line(
              points={{70,-8},{70,-16},{48,-16},{48,44},{34,44},{34,52}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.OutPort, Joint2.InPortB) annotation (Line(
              points={{70,-8},{70,-16},{48,-16},{48,4},{34,4},{34,12}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.OutPort, Joint1.InPortB) annotation (Line(
              points={{70,-8},{70,-36},{34,-36},{34,-28}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.OutPort, Joint0.InPortB) annotation (Line(
              points={{70,-8},{70,-76},{34,-76},{34,-68}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact2.OutPortB, Roller2.InPort) annotation (Line(
              points={{-46,28},{-46,36},{-14,36},{-14,27.9}},
              color={0,0,255},
              smooth=Smooth.None));
        end AutonomousPointContactOmniWheelTest;

        model OmniWheel
          parameter Integer n = 4 "Number of rollers";
          parameter Real alpha = Modelica.Constants.pi/n
            "Max angle of the half-sector";
          parameter Real R = 1 "Omni wheel outer radius";
          parameter Real R1 = R*cos(alpha) "Omni wheel inner radius";
          parameter Real[3] r0 = {0, R, 0};
          parameter Real[4] q0 = {1, 0, 0, 0};
          parameter Real[3] v0 = {1, 0, 0};
          parameter Real[3] omega0 = {0, 0, -1};
          parameter Real[3, 3] T0 = identity(3);
          parameter Real pi = Modelica.Constants.pi;
          parameter SI.Acceleration[3] Gravity = {0, -1, 0};

          annotation (Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                    -100},{100,100}}),
                                 graphics),
            experiment(
              StopTime=40,
              NumberOfIntervals=50000,
              Tolerance=1e-008),
            experimentSetupOutput,
            Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                    {100,100}}), graphics={Bitmap(
                  extent={{70,-70},{-70,70}},
                  imageSource=
                      "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"
                       +
                      "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", 

                  fileName="OmniWheel.PNG")}));

          RollerPointContactForces Contact0 
            annotation (Placement(transformation(extent={{-60,-70},{-40,-50}})));
          TwoPortsHeavyBody Roller0(
            Gravity = Gravity,
            r(start = r0 + T0*{0, -R1, 0}),
            v(start = v0 + T0*cross(omega0, {0, -R1, 0})),
            q(start = QMult(q0, {1, 0, 0, 0})),
            omega(start = {0, omega0[2], omega0[3]} + {0, 0, 0})) 
            annotation (Placement(transformation(extent={{-30,-70},{-10,-50}})));

          RollerPointContactForces Contact1 
            annotation (Placement(transformation(extent={{-60,-30},{-40,-10}})));
          TwoPortsHeavyBody Roller1(
            Gravity = Gravity,
            r(start = r0 + T0*{R1, 0, 0}),
            v(start = v0 + T0*cross(omega0, {R1, 0, 0})),
            q(start = QMult(q0, {cos(pi/4), 0, 0, sin(pi/4)})),
            omega(start = {omega0[2], 0, omega0[3]} + {0, 0, 0})) 
            annotation (Placement(transformation(extent={{-30,-30},{-10,-10}})));

          RollerPointContactForces Contact2 
            annotation (Placement(transformation(extent={{-60,10},{-40,30}})));
          TwoPortsHeavyBody Roller2(
            Gravity = Gravity,
            r(start = r0 + T0*{0, R1, 0}),
            v(start = v0 + T0*cross(omega0, {0, R1, 0})),
            q(start = QMult(q0, {cos(pi/2), 0, 0, sin(pi/2)})),
            omega(start = {0, -omega0[2], omega0[3]} + {0, 0, 0})) 
            annotation (Placement(transformation(extent={{-30,10},{-10,30}})));

          RollerPointContactForces Contact3 
            annotation (Placement(transformation(extent={{-60,50},{-40,70}})));
          TwoPortsHeavyBody Roller3(
            Gravity = Gravity,
            r(start = r0 + T0*{-R1, 0, 0}),
            v(start = v0 + T0*cross(omega0, {-R1, 0, 0})),
            q(start = QMult(q0, {cos(3*pi/4), 0, 0, sin(3*pi/4)})),
            omega(start = {-omega0[2], 0, omega0[3]} + {0, 0, 0})) 
            annotation (Placement(transformation(extent={{-30,50},{-10,70}})));
        //  Rigid Joint3(
        //  FixedJoint Joint3(
        //  SpringJoint Joint3(
          FixedJoint Joint3(
            nA = {1, 0, 0},
            nB = {0, -1, 0},
            rA = {0, 0, 0},
            rB = {-R1, 0, 0}) 
            annotation (Placement(transformation(extent={{0,50},{20,70}})));
        //  Rigid Joint2(
        //  SpringJoint Joint2(
        //  FixedJoint Joint2(
          FixedJoint Joint2(
            nA = {1, 0, 0},
            nB = {-1, 0, 0},
            rA = {0, 0, 0},
            rB = {0, R1, 0}) 
            annotation (Placement(transformation(extent={{0,10},{20,30}})));
        //  Rigid Joint1(
        //  SpringJoint Joint1(
        //  FixedJoint Joint1(
          FixedJoint Joint1(
            nA = {1, 0, 0},
            nB = {0, 1, 0},
            rA = {0, 0, 0},
            rB = {R1, 0, 0}) 
            annotation (Placement(transformation(extent={{0,-30},{20,-10}})));
        //  Rigid Joint0(
        //  SpringJoint Joint0(
        //  FixedJoint Joint0(
          FixedJoint Joint0(
            nA = {1, 0, 0},
            nB = {1, 0, 0},
            rA = {0, 0, 0},
            rB = {0, -R1, 0}) 
            annotation (Placement(transformation(extent={{0,-70},{20,-50}})));
          FivePortsHeavyBody Wheel(
            Gravity = Gravity,
            r(start = r0),
            v(start = v0),
            q(start = q0),
            omega(start = omega0)) 
            annotation (Placement(transformation(extent={{40,-10},{62,10}})));
          KinematicPort InPortK 
            annotation (Placement(transformation(extent={{-50,-100},{-30,-80}})));
          WrenchPort InPortF 
            annotation (Placement(transformation(extent={{30,80},{50,100}})));
          KinematicPort OutPortK 
            annotation (Placement(transformation(extent={{30,-100},{50,-80}})));
          Real[3] w;
        equation
          w = transpose(Wheel.OutPort.T)*(Roller0.r - Wheel.r);
          connect(Contact0.InPortB, Roller0.OutPort) 
            annotation (Line(
              points={{-46,-68},{-46,-76},{-20,-76},{-20,-68}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact0.OutPortB, Roller0.InPort) 
            annotation (Line(
              points={{-46,-52},{-46,-44},{-24,-44},{-24,-52.1}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact1.InPortB, Roller1.OutPort) annotation (Line(
              points={{-46,-28},{-46,-36},{-20,-36},{-20,-28}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact1.OutPortB, Roller1.InPort) annotation (Line(
              points={{-46,-12},{-46,-4},{-24,-4},{-24,-12.1}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact2.InPortB, Roller2.OutPort) annotation (Line(
              points={{-46,12},{-46,4},{-20,4},{-20,12}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact2.OutPortB, Roller2.InPort) annotation (Line(
              points={{-46,28},{-46,36},{-24,36},{-24,27.9}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact3.InPortB, Roller3.OutPort) annotation (Line(
              points={{-46,52},{-46,44},{-20,44},{-20,52}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact3.OutPortB, Roller3.InPort) annotation (Line(
              points={{-46,68},{-46,76},{-24,76},{-24,67.9}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Roller3.InPort1, Joint3.OutPortA) annotation (Line(
              points={{-16,68},{-16,76},{6,76},{6,68}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Roller2.InPort1, Joint2.OutPortA) annotation (Line(
              points={{-16,28},{-16,36},{6,36},{6,28}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Roller1.InPort1, Joint1.OutPortA) annotation (Line(
              points={{-16,-12},{-16,-4},{6,-4},{6,-12}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Roller2.OutPort, Joint2.InPortA) annotation (Line(
              points={{-20,12},{-20,4},{6,4},{6,12}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Roller3.OutPort, Joint3.InPortA) annotation (Line(
              points={{-20,52},{-20,44},{6,44},{6,52}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Roller0.InPort1, Joint0.OutPortA) annotation (Line(
              points={{-16,-52},{-16,-44},{6,-44},{6,-52}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Roller0.OutPort, Joint0.InPortA) annotation (Line(
              points={{-20,-68},{-20,-76},{6,-76},{6,-68}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Roller1.OutPort, Joint1.InPortA) annotation (Line(
              points={{-20,-28},{-20,-36},{6,-36},{6,-28}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.InPort, Joint0.OutPortB) annotation (Line(
              points={{43.74,8},{44,8},{44,14},{36,14},{36,-44},{14,-44},{14,
                  -52}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.InPort2, Joint2.OutPortB) annotation (Line(
              points={{53.42,8},{53.42,36},{14,36},{14,28}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.InPort1, Joint1.OutPortB) annotation (Line(
              points={{48.58,8},{48,8},{48,18},{32,18},{32,-4},{14,-4},{14,-12}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.InPort3, Joint3.OutPortB) annotation (Line(
              points={{58.26,8},{56,8},{56,76},{14,76},{14,68}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.OutPort, Joint3.InPortB) annotation (Line(
              points={{51,-8},{51,-16},{28,-16},{28,44},{14,44},{14,52}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.OutPort, Joint2.InPortB) annotation (Line(
              points={{51,-8},{51,-16},{28,-16},{28,4},{14,4},{14,12}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.OutPort, Joint1.InPortB) annotation (Line(
              points={{51,-8},{51,-36},{14,-36},{14,-28}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.OutPort, Joint0.InPortB) annotation (Line(
              points={{51,-8},{51,-76},{14,-76},{14,-68}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.InPort4, InPortF) annotation (Line(
              points={{59.8,0},{70,0},{70,90},{40,90}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.OutPort, OutPortK) annotation (Line(
              points={{51,-8},{51,-76},{40,-76},{40,-90}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact1.InPortA, InPortK) annotation (Line(
              points={{-54,-28},{-54,-36},{-70,-36},{-70,-90},{-40,-90}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact0.InPortA, InPortK) annotation (Line(
              points={{-54,-68},{-56,-68},{-56,-90},{-40,-90}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact2.InPortA, InPortK) annotation (Line(
              points={{-54,12},{-54,4},{-70,4},{-70,-90},{-40,-90}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact3.InPortA, InPortK) annotation (Line(
              points={{-54,52},{-54,44},{-70,44},{-70,-90},{-40,-90}},
              color={0,0,255},
              smooth=Smooth.None));
        end OmniWheel;

        model OmniWheelGeneral
          parameter Integer n = 4 "Number of rollers";
          parameter Real alpha = Modelica.Constants.pi/n
            "Max angle of the half-sector";
          parameter Real R = 1 "Omni wheel outer radius";
          parameter Real R1 = R*cos(alpha) "Omni wheel inner radius";
          parameter Real[3] r0 = {0, R, 0};
          parameter Real[4] q0 = {1, 0, 0, 0};
          parameter Real[3] v0 = {1, 0, 0};
          parameter Real[3] omega0 = {0, 0, -1};
          parameter Real[3, 3] T0 = identity(3);
          parameter Real pi = Modelica.Constants.pi;
          parameter SI.Acceleration[3] Gravity = {0, -1, 0};

          annotation (Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                    -100},{100,100}}),
                                 graphics),
            experiment(
              StopTime=40,
              NumberOfIntervals=50000,
              Tolerance=1e-008),
            experimentSetupOutput,
            Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                    {100,100}}), graphics={Bitmap(
                  extent={{70,-70},{-70,70}},
                  imageSource=
                      "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"
                       +
                      "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", 

                  fileName="OmniWheel.PNG")}));

          RollerPointContactForcesGeneral Contact0 
            annotation (Placement(transformation(extent={{-60,-70},{-40,-50}})));
          TwoPortsHeavyBody Roller0(
            Gravity = Gravity,
            r(start = r0 + T0*{0, -R1, 0}),
            v(start = v0 + T0*cross(omega0, {0, -R1, 0})),
            q(start = QMult(q0, {1, 0, 0, 0})),
            omega(start = {0, omega0[2], omega0[3]} + {0, 0, 0})) 
            annotation (Placement(transformation(extent={{-30,-70},{-10,-50}})));

          RollerPointContactForcesGeneral Contact1 
            annotation (Placement(transformation(extent={{-60,-30},{-40,-10}})));
          TwoPortsHeavyBody Roller1(
            Gravity = Gravity,
            r(start = r0 + T0*{R1, 0, 0}),
            v(start = v0 + T0*cross(omega0, {R1, 0, 0})),
            q(start = QMult(q0, {cos(pi/4), 0, 0, sin(pi/4)})),
            omega(start = {omega0[2], 0, omega0[3]} + {0, 0, 0})) 
            annotation (Placement(transformation(extent={{-30,-30},{-10,-10}})));

          RollerPointContactForcesGeneral Contact2 
            annotation (Placement(transformation(extent={{-60,10},{-40,30}})));
          TwoPortsHeavyBody Roller2(
            Gravity = Gravity,
            r(start = r0 + T0*{0, R1, 0}),
            v(start = v0 + T0*cross(omega0, {0, R1, 0})),
            q(start = QMult(q0, {cos(pi/2), 0, 0, sin(pi/2)})),
            omega(start = {0, -omega0[2], omega0[3]} + {0, 0, 0})) 
            annotation (Placement(transformation(extent={{-30,10},{-10,30}})));

          RollerPointContactForcesGeneral Contact3 
            annotation (Placement(transformation(extent={{-60,50},{-40,70}})));
          TwoPortsHeavyBody Roller3(
            Gravity = Gravity,
            r(start = r0 + T0*{-R1, 0, 0}),
            v(start = v0 + T0*cross(omega0, {-R1, 0, 0})),
            q(start = QMult(q0, {cos(3*pi/4), 0, 0, sin(3*pi/4)})),
            omega(start = {-omega0[2], 0, omega0[3]} + {0, 0, 0})) 
            annotation (Placement(transformation(extent={{-30,50},{-10,70}})));
        //  Rigid Joint3(
        //  FixedJoint Joint3(
        //  SpringJoint Joint3(
          FixedJoint Joint3(
            nA = {1, 0, 0},
            nB = {0, -1, 0},
            rA = {0, 0, 0},
            rB = {-R1, 0, 0}) 
            annotation (Placement(transformation(extent={{0,50},{20,70}})));
        //  Rigid Joint2(
        //  SpringJoint Joint2(
        //  FixedJoint Joint2(
          FixedJoint Joint2(
            nA = {1, 0, 0},
            nB = {-1, 0, 0},
            rA = {0, 0, 0},
            rB = {0, R1, 0}) 
            annotation (Placement(transformation(extent={{0,10},{20,30}})));
        //  Rigid Joint1(
        //  SpringJoint Joint1(
        //  FixedJoint Joint1(
          FixedJoint Joint1(
            nA = {1, 0, 0},
            nB = {0, 1, 0},
            rA = {0, 0, 0},
            rB = {R1, 0, 0}) 
            annotation (Placement(transformation(extent={{0,-30},{20,-10}})));
        //  Rigid Joint0(
        //  SpringJoint Joint0(
        //  FixedJoint Joint0(
          FixedJoint Joint0(
            nA = {1, 0, 0},
            nB = {1, 0, 0},
            rA = {0, 0, 0},
            rB = {0, -R1, 0}) 
            annotation (Placement(transformation(extent={{0,-70},{20,-50}})));
          FivePortsHeavyBody Wheel(
            Gravity = Gravity,
            r(start = r0),
            v(start = v0),
            q(start = q0),
            omega(start = omega0)) 
            annotation (Placement(transformation(extent={{40,-10},{62,10}})));
          KinematicPort InPortK 
            annotation (Placement(transformation(extent={{-50,-100},{-30,-80}})));
          WrenchPort InPortF 
            annotation (Placement(transformation(extent={{30,80},{50,100}})));
          KinematicPort OutPortK 
            annotation (Placement(transformation(extent={{30,-100},{50,-80}})));
          Real[3] w;
        equation
          Contact0.n1k = Wheel.T*{0, 0, 1};
          Contact1.n1k = Wheel.T*{0, 0, 1};
          Contact2.n1k = Wheel.T*{0, 0, 1};
          Contact3.n1k = Wheel.T*{0, 0, 1};
          Contact0.rho = (Wheel.r - Roller0.r)/sqrt((Wheel.r - Roller0.r)*(Wheel.r - Roller0.r));
          Contact1.rho = (Wheel.r - Roller1.r)/sqrt((Wheel.r - Roller1.r)*(Wheel.r - Roller1.r));
          Contact2.rho = (Wheel.r - Roller2.r)/sqrt((Wheel.r - Roller2.r)*(Wheel.r - Roller2.r));
          Contact3.rho = (Wheel.r - Roller3.r)/sqrt((Wheel.r - Roller3.r)*(Wheel.r - Roller3.r));
          w = transpose(Wheel.OutPort.T)*(Roller0.r - Wheel.r);
          connect(Contact0.InPortB, Roller0.OutPort) 
            annotation (Line(
              points={{-46,-68},{-46,-76},{-20,-76},{-20,-68}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact0.OutPortB, Roller0.InPort) 
            annotation (Line(
              points={{-46,-52},{-46,-44},{-24,-44},{-24,-52.1}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact1.InPortB, Roller1.OutPort) annotation (Line(
              points={{-46,-28},{-46,-36},{-20,-36},{-20,-28}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact1.OutPortB, Roller1.InPort) annotation (Line(
              points={{-46,-12},{-46,-4},{-24,-4},{-24,-12.1}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact2.InPortB, Roller2.OutPort) annotation (Line(
              points={{-46,12},{-46,4},{-20,4},{-20,12}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact2.OutPortB, Roller2.InPort) annotation (Line(
              points={{-46,28},{-46,36},{-24,36},{-24,27.9}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact3.InPortB, Roller3.OutPort) annotation (Line(
              points={{-46,52},{-46,44},{-20,44},{-20,52}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact3.OutPortB, Roller3.InPort) annotation (Line(
              points={{-46,68},{-46,76},{-24,76},{-24,67.9}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Roller3.InPort1, Joint3.OutPortA) annotation (Line(
              points={{-16,68},{-16,76},{6,76},{6,68}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Roller2.InPort1, Joint2.OutPortA) annotation (Line(
              points={{-16,28},{-16,36},{6,36},{6,28}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Roller1.InPort1, Joint1.OutPortA) annotation (Line(
              points={{-16,-12},{-16,-4},{6,-4},{6,-12}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Roller2.OutPort, Joint2.InPortA) annotation (Line(
              points={{-20,12},{-20,4},{6,4},{6,12}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Roller3.OutPort, Joint3.InPortA) annotation (Line(
              points={{-20,52},{-20,44},{6,44},{6,52}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Roller0.InPort1, Joint0.OutPortA) annotation (Line(
              points={{-16,-52},{-16,-44},{6,-44},{6,-52}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Roller0.OutPort, Joint0.InPortA) annotation (Line(
              points={{-20,-68},{-20,-76},{6,-76},{6,-68}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Roller1.OutPort, Joint1.InPortA) annotation (Line(
              points={{-20,-28},{-20,-36},{6,-36},{6,-28}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.InPort, Joint0.OutPortB) annotation (Line(
              points={{43.74,8},{44,8},{44,14},{36,14},{36,-44},{14,-44},{14,
                  -52}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.InPort2, Joint2.OutPortB) annotation (Line(
              points={{53.42,8},{53.42,36},{14,36},{14,28}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.InPort1, Joint1.OutPortB) annotation (Line(
              points={{48.58,8},{48,8},{48,18},{32,18},{32,-4},{14,-4},{14,-12}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.InPort3, Joint3.OutPortB) annotation (Line(
              points={{58.26,8},{58,8},{58,76},{14,76},{14,68}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.OutPort, Joint3.InPortB) annotation (Line(
              points={{51,-8},{51,-16},{28,-16},{28,44},{14,44},{14,52}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.OutPort, Joint2.InPortB) annotation (Line(
              points={{51,-8},{51,-16},{28,-16},{28,4},{14,4},{14,12}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.OutPort, Joint1.InPortB) annotation (Line(
              points={{51,-8},{51,-36},{14,-36},{14,-28}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.OutPort, Joint0.InPortB) annotation (Line(
              points={{51,-8},{51,-76},{14,-76},{14,-68}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.InPort4, InPortF) annotation (Line(
              points={{59.8,0},{70,0},{70,90},{40,90}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel.OutPort, OutPortK) annotation (Line(
              points={{51,-8},{51,-76},{40,-76},{40,-90}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact1.InPortA, InPortK) annotation (Line(
              points={{-54,-28},{-54,-36},{-70,-36},{-70,-90},{-40,-90}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact0.InPortA, InPortK) annotation (Line(
              points={{-54,-68},{-56,-68},{-56,-90},{-40,-90}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact2.InPortA, InPortK) annotation (Line(
              points={{-54,12},{-54,4},{-70,4},{-70,-90},{-40,-90}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Contact3.InPortA, InPortK) annotation (Line(
              points={{-54,52},{-54,44},{-70,44},{-70,-90},{-40,-90}},
              color={0,0,255},
              smooth=Smooth.None));
        end OmniWheelGeneral;

        model AutonomousPatchContactOmniWheelSetTest
          //for debugging
          parameter Integer precisionLevel_v =     6;
          parameter Integer precisionLevel_omega = 6;

          parameter Real platformMass = 10;
          parameter Integer n = 4 "Number of rollers";
          parameter Real alpha = Modelica.Constants.pi/n
            "Max angle of the half-sector";
          parameter Real R = 1 "Omni wheel outer radius";
          parameter Real d = 2 "vehicle size";
          parameter Real R1 = R*cos(alpha) "Omni wheel inner radius";
          parameter Real om0 = 0.1;
          parameter Real[3] r0 = {0, R, 0};
          parameter Real[4] q0 = {1, 0, 0, 0};
          parameter Real[3] v0 = {0, 0, 0};
          parameter Real[3] omega0 = {0, om0, 0};
          parameter Real pi = Modelica.Constants.pi;

        //  parameter Real[3] omega1_0 = omega0 + (-d)*om0/R * {0, 0, 1} + ((-d)*v0*{cos(-pi/2 + 0*2*pi/3), 0, sin(-pi/2 + 0*2*pi/3)}) * {0,0,1};
        //  parameter Real[3] omega2_0 = omega0 + (-d)*om0/R * {0, 0, 1} + ((-d)*v0*{cos(-pi/2 + 1*2*pi/3), 0, sin(-pi/2 + 1*2*pi/3)}) * {0,0,1};
        //  parameter Real[3] omega3_0 = omega0 + (-d)*om0/R * {0, 0, 1} + ((-d)*v0*{cos(-pi/2 + 2*2*pi/3), 0, sin(-pi/2 + 2*2*pi/3)}) * {0,0,1};
          parameter Real[3] omega1_0 = omega0 + (-d)*om0/R * {0, 0, 1};
          parameter Real[3] omega2_0 = omega0 + (-d)*om0/R * {0, 0, 1};
          parameter Real[3] omega3_0 = omega0 + (-d)*om0/R * {0, 0, 1};

          // for plotting:
          Real _angle(start = 0);
          Real _omega0(start = om0);
          Real _omega1(start = sqrt(omega1_0*omega1_0));
          Real _omega2(start = sqrt(omega2_0*omega2_0));
          Real _omega3(start = sqrt(omega3_0*omega3_0));
          Real _r1( start = r0[1]);
          Real _r3( start = r0[3]);
          Real _v1( start = v0[1]);
          Real _v3( start = v0[3]);

        //  Real[3] d;
          Base Floor 
                    annotation (Placement(transformation(extent={{-90,-12},{-70,
                    10}})));
          annotation (Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                    -100},{100,100}}),
                                 graphics),
            experiment(
              StopTime=6,
              NumberOfIntervals=30000,
              Tolerance=1e-009,
              Algorithm="Dassl"),
            experimentSetupOutput);
          FixedJoint Joint1(
            nA = {0, 0, 1},
            nB = {1, 0, 0},
            rA = {0, 0, 0},
            rB = {d, 0, 0}) 
            annotation (Placement(transformation(extent={{20,40},{40,60}})));
          FixedJoint Joint2(
            nA = {0, 0, 1},
            nB = {-cos(pi/3), 0, -cos(pi/6)},
            rA = {0, 0, 0},
            rB = {-d*cos(pi/3), 0, -d*cos(pi/6)}) 
            annotation (Placement(transformation(extent={{20,-10},{40,10}})));
          FixedJoint Joint3(
            nA = {0, 0, 1},
            nB = {-cos(pi/3), 0, cos(pi/6)},
            rA = {0, 0, 0},
            rB = {-d*cos(pi/3), 0, d*cos(pi/6)}) 
            annotation (Placement(transformation(extent={{20,-60},{40,-40}})));
          ThreePortsHeavyBody Platform(
            m = platformMass,
            Gravity = {0, -1, 0},
            r(start = r0),
            v(start = v0),
            q(start = q0),
            omega(start = omega0)) 
            annotation (Placement(transformation(extent={{70,-10},{90,10}})));
          OmniWheel Wheel1(
            Gravity={0,-1,0},
            T0=[0,0,-1; 0,1,0; 1,0,0],
            r0=r0 + {d,0,0},
            v0=v0 + cross(omega0, {d,0,0}),
            q0=QMult(q0, {cos(pi/4),0,sin(pi/4),0}),
            omega0=omega1_0) 
            annotation (Placement(transformation(extent={{-44,36},{-16,64}})));
          OmniWheel Wheel2(
            Gravity={0,-1,0},
            T0=[-cos(pi/6),0,cos(pi/3); 0,1,0; -cos(pi/3),0,-cos(pi/6)],
            r0=r0 + {-d*cos(pi/3),0,-d*cos(pi/6)},
            v0=v0 + cross(omega0, {-d*cos(pi/3),0,-d*cos(pi/6)}),
            q0=QMult(q0, {cos(7*pi/12),0,sin(7*pi/12),0}),
            omega0=omega2_0) 
            annotation (Placement(transformation(extent={{-44,-14},{-16,14}})));
          OmniWheel Wheel3(
            Gravity={0,-1,0},
            T0=[cos(pi/6),0,cos(pi/3); 0,1,0; -cos(pi/3),0,cos(pi/6)],
            r0=r0 + {-d*cos(pi/3),0,d*cos(pi/6)},
            v0=v0 + cross(omega0, {-d*cos(pi/3),0,d*cos(pi/6)}),
            q0=QMult(q0, {cos(-pi/12),0,sin(-pi/12),0}),
            omega0=omega3_0) 
            annotation (Placement(transformation(extent={{-44,-64},{-16,-36}})));
        //    Real w;
        //    Real w1;
        equation

          der(_angle) = _omega0;
          _omega0 = Platform.omega[2];
          _omega1 = sqrt(Wheel1.Wheel.omega*Wheel1.Wheel.omega);
          _omega2 = sqrt(Wheel2.Wheel.omega*Wheel2.Wheel.omega);
          _omega3 = sqrt(Wheel3.Wheel.omega*Wheel3.Wheel.omega);
          _r1 = Platform.r[1];
          _r3 = Platform.r[3];
          _v1 = Platform.v[1];
          _v3 = Platform.v[3];

          assert(noEvent(Platform.v[2]*Platform.v[2] < 10^(-precisionLevel_v)),  "Platform has vertical speed !!!");
          assert(noEvent(Platform.omega[1]*Platform.omega[1] + Platform.omega[3]*Platform.omega[3] < 10^(-precisionLevel_omega)),  "Platform.omega is not all [2] !!!");

        //  w = (Platform.OutPort.T*{0,1,0})*(Wheel1.Wheel.OutPort.T*{0,0,1});
        //  w1 = (Wheel1.Roller0.OutPort.T*{1,0,0})*(Wheel1.Wheel.OutPort.T*{0,0,1});
        //  w = {0,1,0}*(Wheel2.Wheel.OutPort.T*{0,0,1});
        //  w = (Platform.OutPort.T*{0,1,0})*(Wheel2.Wheel.OutPort.T*{0,0,1});
        //  d = cross({0, 1, 0}, Wheel.OutPort.T*{0, 0, 1});
        //  Wheel.OutPort.epsilon*d/sqrt(d*d) = 0;
          connect(Floor.OutPort, Wheel2.InPortK) annotation (Line(
              points={{-80,-9.8},{-80,-26},{-35.6,-26},{-35.6,-12.6}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel1.OutPortK,Joint1. InPortA) annotation (Line(
              points={{-24.4,37.4},{-24.4,30},{26,30},{26,42}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel2.OutPortK, Joint2.InPortA) annotation (Line(
              points={{-24.4,-12.6},{-24.4,-26},{26,-26},{26,-8}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel1.InPortF,Joint1. OutPortA) annotation (Line(
              points={{-24.4,62.6},{-24.4,74},{26,74},{26,58}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel2.InPortF, Joint2.OutPortA) annotation (Line(
              points={{-24.4,12.6},{-24.4,26},{26,26},{26,8}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Platform.OutPort,Joint1. InPortB) annotation (Line(
              points={{80,-8},{80,-20},{60,-20},{60,34},{34,34},{34,42}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Platform.OutPort, Joint2.InPortB) annotation (Line(
              points={{80,-8},{80,-26},{34,-26},{34,-8}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Joint2.OutPortB, Platform.InPort1) annotation (Line(
              points={{34,8},{34,20},{80,20},{80,8}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Joint1.OutPortB, Platform.InPort) annotation (Line(
              points={{34,58},{34,70},{74,70},{74,8}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Floor.OutPort, Wheel3.InPortK) annotation (Line(
              points={{-80,-9.8},{-80,-70},{-35.6,-70},{-35.6,-62.6}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel1.InPortK, Floor.OutPort) annotation (Line(
              points={{-35.6,37.4},{-35.6,30},{-60,30},{-60,-20},{-80,-20},{-80,
                  -9.8}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel3.OutPortK, Joint3.InPortA) annotation (Line(
              points={{-24.4,-62.6},{-24.4,-70},{26,-70},{26,-58}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Joint3.InPortB, Platform.OutPort) annotation (Line(
              points={{34,-58},{34,-70},{80,-70},{80,-8}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel3.InPortF, Joint3.OutPortA) annotation (Line(
              points={{-24.4,-37.4},{-24.4,-30},{26,-30},{26,-42}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Joint3.OutPortB, Platform.InPort2) annotation (Line(
              points={{34,-42},{34,-30},{66,-30},{66,30},{86,30},{86,8}},
              color={0,0,255},
              smooth=Smooth.None));
        end AutonomousPatchContactOmniWheelSetTest;

        model AutonomousPatchContactOmniWheelSetGeneralTest
          //for debugging
          parameter Integer precisionLevel_v =     6;
          parameter Integer precisionLevel_omega = 6;

          parameter Real platformMass = 10;
          parameter Integer n = 4 "Number of rollers";
          parameter Real alpha = Modelica.Constants.pi/n
            "Max angle of the half-sector";
          parameter Real R = 1 "Omni wheel outer radius";
          parameter Real d = 2 "vehicle size";
          parameter Real R1 = R*cos(alpha) "Omni wheel inner radius";
          parameter Real om0 = 0.1;
          parameter Real[3] r0 = {0, R, 0};
          parameter Real[4] q0 = {1, 0, 0, 0};
          parameter Real[3] v0 = {0, 0, 0};
          parameter Real[3] omega0 = {0, om0, 0};
          parameter Real pi = Modelica.Constants.pi;

        //  parameter Real[3] omega1_0 = omega0 + (-d)*om0/R * {0, 0, 1} + ((-d)*v0*{cos(-pi/2 + 0*2*pi/3), 0, sin(-pi/2 + 0*2*pi/3)}) * {0,0,1};
        //  parameter Real[3] omega2_0 = omega0 + (-d)*om0/R * {0, 0, 1} + ((-d)*v0*{cos(-pi/2 + 1*2*pi/3), 0, sin(-pi/2 + 1*2*pi/3)}) * {0,0,1};
        //  parameter Real[3] omega3_0 = omega0 + (-d)*om0/R * {0, 0, 1} + ((-d)*v0*{cos(-pi/2 + 2*2*pi/3), 0, sin(-pi/2 + 2*2*pi/3)}) * {0,0,1};
          parameter Real[3] omega1_0 = omega0 + (-d)*om0/R * {0, 0, 1};
          parameter Real[3] omega2_0 = omega0 + (-d)*om0/R * {0, 0, 1};
          parameter Real[3] omega3_0 = omega0 + (-d)*om0/R * {0, 0, 1};

          // for plotting:
          Real _angle(start = 0);
          Real _omega0(start = om0);
          Real _omega1(start = sqrt(omega1_0*omega1_0));
          Real _omega2(start = sqrt(omega2_0*omega2_0));
          Real _omega3(start = sqrt(omega3_0*omega3_0));
          Real _r1( start = r0[1]);
          Real _r3( start = r0[3]);
          Real _v1( start = v0[1]);
          Real _v3( start = v0[3]);

        //  Real[3] d;
          Base Floor 
                    annotation (Placement(transformation(extent={{-90,-12},{-70,
                    10}})));
          annotation (Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                    -100},{100,100}}),
                                 graphics),
            experiment(
              StopTime=6,
              NumberOfIntervals=30000,
              Tolerance=1e-009,
              Algorithm="Dassl"),
            experimentSetupOutput);
          FixedJoint Joint1(
            nA = {0, 0, 1},
            nB = {1, 0, 0},
            rA = {0, 0, 0},
            rB = {d, 0, 0}) 
            annotation (Placement(transformation(extent={{20,40},{40,60}})));
          FixedJoint Joint2(
            nA = {0, 0, 1},
            nB = {-cos(pi/3), 0, -cos(pi/6)},
            rA = {0, 0, 0},
            rB = {-d*cos(pi/3), 0, -d*cos(pi/6)}) 
            annotation (Placement(transformation(extent={{20,-10},{40,10}})));
          FixedJoint Joint3(
            nA = {0, 0, 1},
            nB = {-cos(pi/3), 0, cos(pi/6)},
            rA = {0, 0, 0},
            rB = {-d*cos(pi/3), 0, d*cos(pi/6)}) 
            annotation (Placement(transformation(extent={{20,-60},{40,-40}})));
          ThreePortsHeavyBody Platform(
            m = platformMass,
            Gravity = {0, -1, 0},
            r(start = r0),
            v(start = v0),
            q(start = q0),
            omega(start = omega0)) 
            annotation (Placement(transformation(extent={{70,-10},{90,10}})));
          OmniWheelGeneral Wheel1(
            Gravity={0,-1,0},
            T0=[0,0,-1; 0,1,0; 1,0,0],
            r0=r0 + {d,0,0},
            v0=v0 + cross(omega0, {d,0,0}),
            q0=QMult(q0, {cos(pi/4),0,sin(pi/4),0}),
            omega0=omega1_0) 
            annotation (Placement(transformation(extent={{-44,36},{-16,64}})));
          OmniWheelGeneral Wheel2(
            Gravity={0,-1,0},
            T0=[-cos(pi/6),0,cos(pi/3); 0,1,0; -cos(pi/3),0,-cos(pi/6)],
            r0=r0 + {-d*cos(pi/3),0,-d*cos(pi/6)},
            v0=v0 + cross(omega0, {-d*cos(pi/3),0,-d*cos(pi/6)}),
            q0=QMult(q0, {cos(7*pi/12),0,sin(7*pi/12),0}),
            omega0=omega2_0) 
            annotation (Placement(transformation(extent={{-44,-14},{-16,14}})));
          OmniWheelGeneral Wheel3(
            Gravity={0,-1,0},
            T0=[cos(pi/6),0,cos(pi/3); 0,1,0; -cos(pi/3),0,cos(pi/6)],
            r0=r0 + {-d*cos(pi/3),0,d*cos(pi/6)},
            v0=v0 + cross(omega0, {-d*cos(pi/3),0,d*cos(pi/6)}),
            q0=QMult(q0, {cos(-pi/12),0,sin(-pi/12),0}),
            omega0=omega3_0) 
            annotation (Placement(transformation(extent={{-44,-64},{-16,-36}})));
        //    Real w;
        //    Real w1;
        equation

          der(_angle) = _omega0;
          _omega0 = Platform.omega[2];
          _omega1 = sqrt(Wheel1.Wheel.omega*Wheel1.Wheel.omega);
          _omega2 = sqrt(Wheel2.Wheel.omega*Wheel2.Wheel.omega);
          _omega3 = sqrt(Wheel3.Wheel.omega*Wheel3.Wheel.omega);
          _r1 = Platform.r[1];
          _r3 = Platform.r[3];
          _v1 = Platform.v[1];
          _v3 = Platform.v[3];

          assert(noEvent(Platform.v[2]*Platform.v[2] < 10^(-precisionLevel_v)),  "Platform has vertical speed !!!");
          assert(noEvent(Platform.omega[1]*Platform.omega[1] + Platform.omega[3]*Platform.omega[3] < 10^(-precisionLevel_omega)),  "Platform.omega is not all [2] !!!");

        //  w = (Platform.OutPort.T*{0,1,0})*(Wheel1.Wheel.OutPort.T*{0,0,1});
        //  w1 = (Wheel1.Roller0.OutPort.T*{1,0,0})*(Wheel1.Wheel.OutPort.T*{0,0,1});
        //  w = {0,1,0}*(Wheel2.Wheel.OutPort.T*{0,0,1});
        //  w = (Platform.OutPort.T*{0,1,0})*(Wheel2.Wheel.OutPort.T*{0,0,1});
        //  d = cross({0, 1, 0}, Wheel.OutPort.T*{0, 0, 1});
        //  Wheel.OutPort.epsilon*d/sqrt(d*d) = 0;
          connect(Floor.OutPort, Wheel2.InPortK) annotation (Line(
              points={{-80,-9.8},{-80,-26},{-35.6,-26},{-35.6,-12.6}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel1.OutPortK,Joint1. InPortA) annotation (Line(
              points={{-24.4,37.4},{-24.4,30},{26,30},{26,42}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel2.OutPortK, Joint2.InPortA) annotation (Line(
              points={{-24.4,-12.6},{-24.4,-26},{26,-26},{26,-8}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel1.InPortF,Joint1. OutPortA) annotation (Line(
              points={{-24.4,62.6},{-24.4,74},{26,74},{26,58}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel2.InPortF, Joint2.OutPortA) annotation (Line(
              points={{-24.4,12.6},{-24.4,26},{26,26},{26,8}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Platform.OutPort,Joint1. InPortB) annotation (Line(
              points={{80,-8},{80,-20},{60,-20},{60,34},{34,34},{34,42}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Platform.OutPort, Joint2.InPortB) annotation (Line(
              points={{80,-8},{80,-26},{34,-26},{34,-8}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Joint2.OutPortB, Platform.InPort1) annotation (Line(
              points={{34,8},{34,20},{80,20},{80,8}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Joint1.OutPortB, Platform.InPort) annotation (Line(
              points={{34,58},{34,70},{74,70},{74,8}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Floor.OutPort, Wheel3.InPortK) annotation (Line(
              points={{-80,-9.8},{-80,-70},{-35.6,-70},{-35.6,-62.6}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel1.InPortK, Floor.OutPort) annotation (Line(
              points={{-35.6,37.4},{-35.6,30},{-60,30},{-60,-20},{-80,-20},{-80,
                  -9.8}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel3.OutPortK, Joint3.InPortA) annotation (Line(
              points={{-24.4,-62.6},{-24.4,-70},{26,-70},{26,-58}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Joint3.InPortB, Platform.OutPort) annotation (Line(
              points={{34,-58},{34,-70},{80,-70},{80,-8}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Wheel3.InPortF, Joint3.OutPortA) annotation (Line(
              points={{-24.4,-37.4},{-24.4,-30},{26,-30},{26,-42}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(Joint3.OutPortB, Platform.InPort2) annotation (Line(
              points={{34,-42},{34,-30},{66,-30},{66,30},{86,30},{86,8}},
              color={0,0,255},
              smooth=Smooth.None));
        end AutonomousPatchContactOmniWheelSetGeneralTest;

        model NRollersOmniWheel

          import Modelica.Constants.pi;

          parameter Integer NRollers = 4 "Number of rollers";
          parameter Real alpha = pi/NRollers "Max angle of the half-sector";
          parameter Real R = 1 "Omni wheel outer radius";
          parameter Real R1 = R*cos(alpha) "Omni wheel inner radius";
          parameter Real[3] r0 = {0, R, 0};
          parameter Real[4] q0 = {1, 0, 0, 0};
          parameter Real[3] v0 = {1, 0, 0}; //absolute in global
          parameter Real[3] omega0 = {0, 0, -1};
          parameter Real[3, 3] T0 = identity(3); //columns are global frame vectors resolved in local frame, i.e. T transforms from global to local coordinates.

          parameter Real rollerFraction = 10^(-6); //0.3; //fraction of rollers' inertia in that of the whole wheel.
          parameter Real wheelMass = 1;
          parameter Real[3] wheelInertia = {1, 1, 1}; //INERZIA STUPIZY
          //TODO: proper roller mass and inertia
          parameter Real rollerMass = wheelMass*rollerFraction/NRollers;
        //  parameter Real[3] rollerInertia = rollerMass * {1/2*(R-R1)^2, 3/12*(R-R1)^2 + 1/12*(2*R*sin(alpha))^2, 3/12*(R-R1)^2 + 1/12*(2*R*sin(alpha))^2};
          parameter Real[3] rollerInertia = rollerMass * {1/2, 1, 1};

          parameter Real pi = Modelica.Constants.pi;
          parameter SI.Acceleration[3] Gravity = {0, -1, 0};

          parameter Real[NRollers,3] rollerDirs = {{cos(-pi/2 + 2*alpha*(i-1)), sin(-pi/2 + 2*alpha*(i-1)), 0} for i in 1:NRollers};

          // quaternion = {cos(phi/2), i*sin(phi/2), j*sin(), k*sin()}, where phi - angle of clockwise rotation around {i,j,k}
          // os' zed on ekrana na nas, a krutit' nado vokrug osi v ekran protiv chasovoi, naprimer
          parameter Real[NRollers,4] roller_q0s_local = {{cos(-alpha*(i-1)), 0, 0, -1 * sin(-alpha*(i-1))} for i in 1:NRollers};
          parameter Real[NRollers,4] roller_q0s =       {QMult1(q0, roller_q0s_local[i,:]) for i in 1:NRollers};
          parameter Real[NRollers,3,3] qtots =          {QToT(roller_q0s_local[i,:]) for i in 1:NRollers};

          parameter Real[NRollers,3] joint_nB_Dirs = {{cos(2*alpha*(i-1)), sin(2*alpha*(i-1)), 0} for i in 1:NRollers};

          parameter Real[NRollers,3] omega0roller = {(qtots[i,:,:]*omega0 + (if i == 1 then (v0*(transpose(T0)*{0,0,1}))/(R-R1)*{1,0,0} else {0,0,0})) for i in 1:NRollers};

          TwoPortsHeavyBody[NRollers] Rollers(
            each m = rollerMass,
            each I = diagonal(rollerInertia),
            each Gravity = Gravity,
            r(start = {r0 + T0*R1*rollerDirs[i,:] for i in 1:NRollers}),
            v(start = {v0 + T0*cross(omega0, R1*rollerDirs[i,:]) for i in 1:NRollers}),
            omega(start = {omega0roller[i,:] for i in 1:NRollers}),
            q(start = roller_q0s));

          RollerPointContactForces[NRollers] Contacts(
            each n = NRollers);

          FixedJoint[NRollers] Joints(
            nA = {{1, 0, 0} for i in 1:NRollers},
            nB = {joint_nB_Dirs[i,:] for i in 1:NRollers},
            rA = {{0, 0, 0} for i in 1:NRollers},
            rB = {R1 * rollerDirs[i,:] for i in 1:NRollers});

          NPortsHeavyBody Wheel(
            NPorts = NRollers + 1,
            m = wheelMass - NRollers*rollerMass,
            Gravity = Gravity,
            r(start = r0),
            v(start = v0),
            q(start = q0),
            omega(start = omega0),
            I = diagonal(wheelInertia));

          KinematicPort InPortK;
          WrenchPort InPortF;
          KinematicPort OutPortK;

          Real[3] w;

          //assumes that the first roller is in contact!
        //  parameter Real[3] vt0 = v0 + T0*cross(omega0, R1*rollerDirs[1,:]) + T0 * cross(omega0[2] * omega0Dirs[1,:] + {0, 0, omega0[3]} + {0, 0, 0}, (R1-R)*{0, -1, 0});
        //  Real vt(start = sqrt(vt0 * vt0)); //?

          //Assumes ideal rolling at start
          Real _vt(start = 0); //?

          Real _rollerInContactNumber;

        equation
          w = transpose(Wheel.OutPort.T)*(Rollers[1].r - Wheel.r);

          _vt = sum(Contacts[i].relvtsqrt * Contacts[i].isInContact for i in 1:NRollers);
          _rollerInContactNumber = sum(i * Contacts[i].isInContact for i in 1:NRollers);

          for i in 1:NRollers loop
            connect(Contacts[i].InPortA, InPortK);
            connect(Contacts[i].InPortB, Rollers[i].OutPort);
            connect(Contacts[i].OutPortB, Rollers[i].InPort);
            connect(Rollers[i].InPort1, Joints[i].OutPortA);
            connect(Rollers[i].OutPort, Joints[i].InPortA);
            connect(Joints[i].OutPortB, Wheel.InPorts[i]);
            connect(Joints[i].InPortB, Wheel.OutPort);
          end for;

          connect(Wheel.InPorts[NRollers + 1], InPortF);
          connect(Wheel.OutPort, OutPortK);

        end NRollersOmniWheel;

        model NRollersOmniWheelGeneral

          import Modelica.Constants.pi;

          parameter Integer NRollers = 4 "Number of rollers";
          parameter Real psi = 0 "Angle of roller distortion (fixed axis turn)";
          parameter Real alpha = pi/NRollers "Max angle of the half-sector";
          parameter Real R = 1 "Omni wheel outer radius";
          parameter Real R1 = R*cos(alpha) "Omni wheel inner radius";
          parameter Real[3] r0 = {0, R, 0};
          parameter Real[4] q0 = {1, 0, 0, 0};
          parameter Real[3] v0 = {1, 0, 0}; //absolute in global
          parameter Real[3] omega0 = {0, 0, -1};
          parameter Real[3, 3] T0 = identity(3); //columns are global frame vectors resolved in local frame, i.e. T transforms from global to local coordinates.

          parameter Real rollerFraction = 0.3; //fraction of rollers' inertia in that of the whole wheel.
          parameter Real wheelMass = 1;
          parameter Real[3] wheelInertia = {1, 1, 1}; //INERZIA STUPIZY
          //TODO: proper roller mass and inertia
          parameter Real rollerMass = wheelMass*rollerFraction/NRollers;
        //  parameter Real[3] rollerInertia = rollerMass * {1/2*(R-R1)^2, 3/12*(R-R1)^2 + 1/12*(2*R*sin(alpha))^2, 3/12*(R-R1)^2 + 1/12*(2*R*sin(alpha))^2};
          parameter Real[3] rollerInertia = rollerMass * {1/2, 1, 1};

          parameter Real pi = Modelica.Constants.pi;
          parameter SI.Acceleration[3] Gravity = {0, -1, 0};

          // spizy or outer normals
          parameter Real[NRollers,3] rollerDirs = {{cos(-pi/2 + 2*alpha*(i-1)), sin(-pi/2 + 2*alpha*(i-1)), 0} for i in 1:NRollers};

          // Note:
          // quaternion = {cos(phi/2), i*sin(phi/2), j*sin(), k*sin()}, where phi - angle of clockwise rotation around {i,j,k}
          // (looking in same direction as axis)

          // calculating roller axes through psi:
          // turn clockwise around outer normal by psi:

          // os' zed on ekrana na nas, a krutit' nado vokrug osi v ekran protiv chasovoi, naprimer
          parameter Real[4] first_roller_q0_local = {cos(psi/2), 0, sin(psi/2), 0};
          parameter Real[NRollers,4] roller_q0s_local = {QMult1({cos(alpha*(i-1)), 0, 0, sin(alpha*(i-1))}, first_roller_q0_local) for i in 1:NRollers};
          parameter Real[NRollers,4] roller_q0s = {QMult1(q0, roller_q0s_local[i,:]) for i in 1:NRollers};
          // roller coords to wheel coords:
          parameter Real[NRollers,3,3] qtots_local = {QToT(roller_q0s_local[i,:]) for i in 1:NRollers};
          // roller coords to global coords:
          parameter Real[NRollers,3,3] qtots = {QToT(roller_q0s[i,:]) for i in 1:NRollers};

          parameter Real[NRollers,3] joint_nB_Dirs = {qtots[i,:,:]*{1,0,0} for i in 1:NRollers};

        //  parameter Real[NRollers,3] omega0roller = {(qtots[i,:,:]*omega0 + (if i == 1 then (v0*(transpose(T0)*{0,0,1}))/(R-R1)*{1,0,0} else {0,0,0})) for i in 1:NRollers};
          parameter Real[NRollers,3] omega0roller = {(transpose(qtots_local[i,:,:])*omega0) for i in 1:NRollers};

          TwoPortsHeavyBody[NRollers] Rollers(
            each m = rollerMass,
            each I = diagonal(rollerInertia),
            each Gravity = Gravity,
            r(start = {r0 + T0*R1*rollerDirs[i,:] for i in 1:NRollers}),
            v(start = {v0 + T0*cross(omega0, R1*rollerDirs[i,:]) for i in 1:NRollers}),
            omega(start = {omega0roller[i,:] for i in 1:NRollers}),
            q(start = roller_q0s));

          RollerPointContactForcesGeneral[NRollers] Contacts(
            rho(start = {{-sin(2*alpha*(i - 1)), cos(2*alpha*(i - 1)), 0} for i in 1:NRollers}),
            each n = NRollers,
            each psi = psi);

          FixedJoint[NRollers] Joints(
            nA = {{1, 0, 0} for i in 1:NRollers},
            nB = {joint_nB_Dirs[i,:] for i in 1:NRollers},
            rA = {{0, 0, 0} for i in 1:NRollers},
            rB = {R1 * rollerDirs[i,:] for i in 1:NRollers});

          NPortsHeavyBody Wheel(
            NPorts = NRollers + 1,
            m = wheelMass - NRollers*rollerMass,
            Gravity = Gravity,
            r(start = r0),
            v(start = v0),
            q(start = q0),
            omega(start = omega0),
            I = diagonal(wheelInertia));

          KinematicPort InPortK;
          WrenchPort InPortF;
          KinematicPort OutPortK;

          Real[3] w;

          //assumes that the first roller is in contact!
        //  parameter Real[3] vt0 = v0 + T0*cross(omega0, R1*rollerDirs[1,:]) + T0 * cross(omega0[2] * omega0Dirs[1,:] + {0, 0, omega0[3]} + {0, 0, 0}, (R1-R)*{0, -1, 0});
        //  Real vt(start = sqrt(vt0 * vt0)); //?

          //Assumes ideal rolling at start
          Real _vt(start = 0); //?

          Real _rollerInContactNumber;

        equation
          w = transpose(Wheel.OutPort.T)*(Rollers[1].r - Wheel.r);

          _vt = sum(Contacts[i].relvtsqrt * Contacts[i].isInContact for i in 1:NRollers);
          _rollerInContactNumber = sum(i * Contacts[i].isInContact for i in 1:NRollers);

          for i in 1:NRollers loop
            connect(Contacts[i].InPortA, InPortK);
            connect(Contacts[i].InPortB, Rollers[i].OutPort);
            connect(Contacts[i].OutPortB, Rollers[i].InPort);
            connect(Rollers[i].InPort1, Joints[i].OutPortA);
            connect(Rollers[i].OutPort, Joints[i].InPortA);
            connect(Joints[i].OutPortB, Wheel.InPorts[i]);
            connect(Joints[i].InPortB, Wheel.OutPort);
          end for;

          connect(Wheel.InPorts[NRollers + 1], InPortF);
          connect(Wheel.OutPort, OutPortK);

        end NRollersOmniWheelGeneral;

        model NRollersOmniWheelGeneralStep

          import Modelica.Constants.pi;

          parameter Integer NRollers = 4 "Number of rollers";
          parameter Real psi = 0 "Angle of roller distortion (fixed axis turn)";
          parameter Real alpha = pi/NRollers "Max angle of the half-sector";
          parameter Real R = 1 "Omni wheel outer radius";
          parameter Real R1 = R*cos(alpha) "Omni wheel inner radius";
          parameter Real[3] r0 = {0, R, 0};
          parameter Real[4] q0 = {1, 0, 0, 0};
          parameter Real[3] v0 = {1, 0, 0}; //absolute in global
          parameter Real[3] omega0 = {0, 0, -1};
          parameter Real[3, 3] T0 = identity(3); //columns are global frame vectors resolved in local frame, i.e. T transforms from global to local coordinates.

          parameter Real rollerFraction = 0.3; //fraction of rollers' inertia in that of the whole wheel.
          parameter Real wheelMass = 1;
          parameter Real[3] wheelInertia = {1, 1, 1}; //INERZIA STUPIZY
          //TODO: proper roller mass and inertia
          parameter Real rollerMass = wheelMass*rollerFraction/NRollers;
        //  parameter Real[3] rollerInertia = rollerMass * {1/2*(R-R1)^2, 3/12*(R-R1)^2 + 1/12*(2*R*sin(alpha))^2, 3/12*(R-R1)^2 + 1/12*(2*R*sin(alpha))^2};
          parameter Real[3] rollerInertia = rollerMass * {1/2, 1, 1};

          parameter Real pi = Modelica.Constants.pi;
          parameter SI.Acceleration[3] Gravity = {0, -1, 0};

          // spizy or outer normals
          parameter Real[NRollers,3] rollerDirs = {{cos(-pi/2 + 2*alpha*(i-1)), sin(-pi/2 + 2*alpha*(i-1)), 0} for i in 1:NRollers};

          // Note:
          // quaternion = {cos(phi/2), i*sin(phi/2), j*sin(), k*sin()}, where phi - angle of clockwise rotation around {i,j,k}
          // (looking in same direction as axis)

          // calculating roller axes through psi:
          // turn clockwise around outer normal by psi:

          // os' zed on ekrana na nas, a krutit' nado vokrug osi v ekran protiv chasovoi, naprimer
          parameter Real[4] first_roller_q0_local = {cos(psi/2), 0, sin(psi/2), 0};
          parameter Real[NRollers,4] roller_q0s_local = {QMult1({cos(alpha*(i-1)), 0, 0, sin(alpha*(i-1))}, first_roller_q0_local) for i in 1:NRollers};
          parameter Real[NRollers,4] roller_q0s = {QMult1(q0, roller_q0s_local[i,:]) for i in 1:NRollers};
          // roller coords to wheel coords:
          parameter Real[NRollers,3,3] qtots_local = {QToT(roller_q0s_local[i,:]) for i in 1:NRollers};
          // roller coords to global coords:
          parameter Real[NRollers,3,3] qtots = {QToT(roller_q0s[i,:]) for i in 1:NRollers};

          parameter Real[NRollers,3] joint_nB_Dirs = {qtots[i,:,:]*{1,0,0} for i in 1:NRollers};

        //  parameter Real[NRollers,3] omega0roller = {(qtots[i,:,:]*omega0 + (if i == 1 then (v0*(transpose(T0)*{0,0,1}))/(R-R1)*{1,0,0} else {0,0,0})) for i in 1:NRollers};
          parameter Real[NRollers,3] omega0roller = {(transpose(qtots_local[i,:,:])*omega0) for i in 1:NRollers};

          TwoPortsHeavyBody[NRollers] Rollers(
            each m = rollerMass,
            each I = diagonal(rollerInertia),
            each Gravity = Gravity,
            r(start = {r0 + T0*R1*rollerDirs[i,:] for i in 1:NRollers}),
            v(start = {v0 + T0*cross(omega0, R1*rollerDirs[i,:]) for i in 1:NRollers}),
            omega(start = {omega0roller[i,:] for i in 1:NRollers}),
            q(start = roller_q0s));

          RollerPointContactForcesGeneralStep[NRollers] Contacts(
            rho(start = {{-sin(2*alpha*(i - 1)), cos(2*alpha*(i - 1)), 0} for i in 1:NRollers}),
            each n = NRollers,
            each psi = psi);

          FixedJoint[NRollers] Joints(
            nA = {{1, 0, 0} for i in 1:NRollers},
            nB = {joint_nB_Dirs[i,:] for i in 1:NRollers},
            rA = {{0, 0, 0} for i in 1:NRollers},
            rB = {R1 * rollerDirs[i,:] for i in 1:NRollers});

          NPortsHeavyBody Wheel(
            NPorts = NRollers + 1,
            m = wheelMass - NRollers*rollerMass,
            Gravity = Gravity,
            r(start = r0),
            v(start = v0),
            q(start = q0),
            omega(start = omega0),
            I = diagonal(wheelInertia));

          KinematicPort InPortK;
          WrenchPort InPortF;
          KinematicPort OutPortK;

          Real[3] w;

          //assumes that the first roller is in contact!
        //  parameter Real[3] vt0 = v0 + T0*cross(omega0, R1*rollerDirs[1,:]) + T0 * cross(omega0[2] * omega0Dirs[1,:] + {0, 0, omega0[3]} + {0, 0, 0}, (R1-R)*{0, -1, 0});
        //  Real vt(start = sqrt(vt0 * vt0)); //?

          //Assumes ideal rolling at start
          Real _vt(start = 0); //?

          Real _rollerInContactNumber;

        equation
          w = transpose(Wheel.OutPort.T)*(Rollers[1].r - Wheel.r);

          _vt = sum(Contacts[i].relvtsqrt * Contacts[i].isInContact for i in 1:NRollers);
          _rollerInContactNumber = sum(i * Contacts[i].isInContact for i in 1:NRollers);

          for i in 1:NRollers loop
            connect(Contacts[i].InPortA, InPortK);
            connect(Contacts[i].InPortB, Rollers[i].OutPort);
            connect(Contacts[i].OutPortB, Rollers[i].InPort);
            connect(Rollers[i].InPort1, Joints[i].OutPortA);
            connect(Rollers[i].OutPort, Joints[i].InPortA);
            connect(Joints[i].OutPortB, Wheel.InPorts[i]);
            connect(Joints[i].InPortB, Wheel.OutPort);
          end for;

          connect(Wheel.InPorts[NRollers + 1], InPortF);
          connect(Wheel.OutPort, OutPortK);

        end NRollersOmniWheelGeneralStep;

        model NRollersPointContactOmniWheelTest

          parameter Integer NRollers = 6 "Number of rollers";
          parameter Real alpha = Modelica.Constants.pi/NRollers
            "Max angle of the half-sector";
          parameter Real R = 1 "Omni wheel outer radius";
          parameter Real R1 = R*cos(alpha) "Omni wheel inner radius";
          parameter Real[3] omega0 = {0, 0, -1} * 2*alpha;
          parameter Real[3] v0 = cross(R*{0, -1, 0}, omega0);

          ForceBase floor;
          NRollersOmniWheel wheel(
            NRollers = NRollers,
            alpha = alpha,
            R = R,
            R1 = R1,
            v0 = v0,
            omega0 = omega0);

          // for plotting:
          Real[3] _r;
          Real[3] _v;
          Real _omega(start = omega0[3]);
          Real _angle(start = 0);

        equation
          connect(wheel.InPortK, floor.OutPort);
          connect(wheel.InPortF, floor.InPort);

          _r = wheel.Wheel.r;
          _v = wheel.Wheel.v;
          _omega = wheel.Wheel.omega[3];
          der(_angle) = _omega;

        end NRollersPointContactOmniWheelTest;

        model NRollersPointContactOmniWheelGeneralTest
          import Modelica.Constants.pi;

          parameter Real _psi = pi/4;

          parameter Integer NRollers = 4 "Number of rollers";
          parameter Real alpha = Modelica.Constants.pi/NRollers
            "Max angle of the half-sector";
          parameter Real R = 1 "Omni wheel outer radius";
          parameter Real R1 = R*cos(alpha) "Omni wheel inner radius";
          parameter Real[3] omega0 = {0, 0, -1} * 2*alpha;
          parameter Real[3] v0 = cross(R*{0, -1, 0}, omega0);

          ForceBase floor;
          NRollersOmniWheelGeneral wheel(
            psi = _psi,
            NRollers = NRollers,
            alpha = alpha,
            R = R,
            R1 = R1,
            v0 = v0,
            omega0 = omega0);

          // for plotting:
          Real[3] _r;
          Real[3] _v;
          Real _omega(start = omega0[3]);
          Real _angle(start = 0);

        equation
          connect(wheel.InPortK, floor.OutPort);

          connect(wheel.InPortF, floor.InPort);
          // Applying constant torque to wheel
        //  wheel.InPortF.F = floor.InPort.F;
        //  wheel.InPortF.M = {0,0,1};
        //  wheel.InPortF.P = floor.InPort.P;

          _r = wheel.Wheel.r;

          _v = wheel.Wheel.v;
          _omega = wheel.Wheel.omega[3];
          der(_angle) = _omega * 180 / pi;

        end NRollersPointContactOmniWheelGeneralTest;

        model NRollersPointContactOmniWheelGeneralVerticalTest
          import Modelica.Constants.pi;

        //  parameter Real _psi = 0;
          parameter Real _psi = 10e-1;//pi/4;
        //  parameter Real _psi = 7e-1;

          parameter Integer NRollers = 4 "Number of rollers";
          parameter Real alpha = Modelica.Constants.pi/NRollers
            "Max angle of the half-sector";
          parameter Real R = 1 "Omni wheel outer radius";
          parameter Real R1 = R*cos(alpha) "Omni wheel inner radius";
          parameter Real initial_spin = 0; //needs checking
          parameter Real[3] omega0 = -1 * {sin(initial_spin), 0, cos(initial_spin)} * 2*alpha;
          parameter Real[4] _q0 = {cos(initial_spin/2), 0, sin(initial_spin/2), 0};
          parameter Real[3] v0 = cross(R*{0, -1, 0}, omega0);

        //  ForceBase floor;
          Base floor;
          NRollersOmniWheelGeneral wheel(
            psi = _psi,
            NRollers = NRollers,
            alpha = alpha,
            R = R,
            R1 = R1,
            v0 = v0,
            q0 = _q0,
            omega0 = omega0);

          // for plotting:
          Real[3] _r;
          Real[3] _v;
          Real _omega(start = omega0[3]);
          Real _angle(start = 0);
          Real lambda(start = 0);
        equation
          wheel.Contacts[1].n1k = {wheel.OutPortK.T[1,3], wheel.OutPortK.T[2,3], wheel.OutPortK.T[3,3]};
          wheel.Contacts[2].n1k = {wheel.OutPortK.T[1,3], wheel.OutPortK.T[2,3], wheel.OutPortK.T[3,3]};
          wheel.Contacts[3].n1k = {wheel.OutPortK.T[1,3], wheel.OutPortK.T[2,3], wheel.OutPortK.T[3,3]};
          wheel.Contacts[4].n1k = {wheel.OutPortK.T[1,3], wheel.OutPortK.T[2,3], wheel.OutPortK.T[3,3]};

          connect(wheel.InPortK, floor.OutPort);

          wheel.OutPortK.T[2,3] = 0;

          wheel.InPortF.F = {0,0,0};
          wheel.InPortF.M = {lambda,0,0};
          wheel.InPortF.P = wheel.Wheel.r;

          _r = wheel.Wheel.r;

          _v = wheel.Wheel.v;
          _omega = wheel.Wheel.omega[3];
          der(_angle) = _omega * 180 / pi;

          annotation (experiment(
              StopTime=10,
              NumberOfIntervals=50000,
              Tolerance=1e-010,
              Algorithm="Dassl"),experimentSetupOutput);
        end NRollersPointContactOmniWheelGeneralVerticalTest;

        model NRollersPointContactOmniWheelGeneralVerticalStepTest
          import Modelica.Constants.pi;

        //  parameter Real _psi = 0;
          parameter Real _psi = 10e-1;//pi/4;
        //  parameter Real _psi = 7e-1;

          parameter Integer NRollers = 4 "Number of rollers";
          parameter Real alpha = Modelica.Constants.pi/NRollers
            "Max angle of the half-sector";
          parameter Real R = 1 "Omni wheel outer radius";
          parameter Real R1 = R*cos(alpha) "Omni wheel inner radius";
          parameter Real initial_spin = 0; //needs checking
          parameter Real[3] omega0 = -1 * {sin(initial_spin), 0, cos(initial_spin)} * 2*alpha;
          parameter Real[4] _q0 = {cos(initial_spin/2), 0, sin(initial_spin/2), 0};
          parameter Real[3] v0 = cross(R*{0, -1, 0}, omega0);
          parameter Real[3] nA = {0, 1, 0} "Vertical";

        //  ForceBase floor;
          Base floor;
          NRollersOmniWheelGeneralStep wheel(
            psi = _psi,
            NRollers = NRollers,
            alpha = alpha,
            R = R,
            R1 = R1,
            v0 = v0,
            q0 = _q0,
            omega0 = omega0);

          Real[3] k1;
          Real[3] j1;
          Real[3] i2;
          Real[3] i1;

          // for plotting:
          Real[3] _r;
          Real[3] _v;
          Real _omega(start = omega0[3]);
          Real _angle(start = 0);
          Real lambda(start = 0);
        equation
          wheel.Contacts[1].n1k = {wheel.OutPortK.T[1,3], wheel.OutPortK.T[2,3], wheel.OutPortK.T[3,3]};
          wheel.Contacts[2].n1k = {wheel.OutPortK.T[1,3], wheel.OutPortK.T[2,3], wheel.OutPortK.T[3,3]};
          wheel.Contacts[3].n1k = {wheel.OutPortK.T[1,3], wheel.OutPortK.T[2,3], wheel.OutPortK.T[3,3]};
          wheel.Contacts[4].n1k = {wheel.OutPortK.T[1,3], wheel.OutPortK.T[2,3], wheel.OutPortK.T[3,3]};
          wheel.Contacts[1].rO = wheel.Wheel.r;
          wheel.Contacts[2].rO = wheel.Wheel.r;
          wheel.Contacts[3].rO = wheel.Wheel.r;
          wheel.Contacts[4].rO = wheel.Wheel.r;

          connect(wheel.InPortK, floor.OutPort);

          wheel.OutPortK.T[2,3] = 0;

          k1 = {wheel.OutPortK.T[1,3], wheel.OutPortK.T[2,3], wheel.OutPortK.T[3,3]};
          j1 = nA;
          i2 = cross(j1, k1);
          i1 = i2/sqrt(i2*i2);
          wheel.InPortF.F = {0,0,0};
          wheel.InPortF.M = lambda*i1;
        //  wheel.InPortF.M = {lambda,0,0};
          wheel.InPortF.P = wheel.Wheel.r;

          _r = wheel.Wheel.r;

          _v = wheel.Wheel.v;
          _omega = wheel.Wheel.omega[3];
          der(_angle) = _omega * 180 / pi;

          annotation (experiment(
              StopTime=10,
              NumberOfIntervals=50000,
              Tolerance=1e-010,
              Algorithm="Dassl"),experimentSetupOutput);
        end NRollersPointContactOmniWheelGeneralVerticalStepTest;

        model ModularPointContactOmniWheelTest

          parameter Integer NRollers = 4 "Number of rollers";
          parameter Real alpha = Modelica.Constants.pi/NRollers
            "Max angle of the half-sector";
          parameter Real R = 1 "Omni wheel outer radius";
          parameter Real R1 = R*cos(alpha) "Omni wheel inner radius";
          parameter Real[3] v0 = {1, 0, 0};
          parameter Real[3] omega0 = {0, 1, -1};

          ForceBase floor;
          OmniWheel wheel(
            alpha = alpha,
            R = R,
            R1 = R1,
            v0 = v0,
            omega0 = omega0);

          // for plotting:
          Real[3] _r;
          Real[3] _v;
          Real _omega;
          Real _angle(start = 0);

        equation
          connect(wheel.InPortK, floor.OutPort);
          connect(wheel.InPortF, floor.InPort);

          _r = wheel.Wheel.r;
          _v = wheel.Wheel.v;
          _omega = wheel.Wheel.omega[2];
          der(_angle) = _omega;

        end ModularPointContactOmniWheelTest;

        model NRollersPointContactOmniWheelSetTest
          import Modelica.Constants.pi;

          //for debugging
          parameter Integer precisionLevel_v =     5;
          parameter Integer precisionLevel_omega = 6;

          // interface for comparing models
          parameter Real platformDiameter = 1;
          parameter Real platformMass = 10;
          parameter Real platformInertia = 10;
          parameter Real wheelMass = 1;
          parameter Real[3] wheelInertia = {1, 1, 1};
          parameter Real rollerFraction = 10^(-6); //0.3; //fraction of rollers' inertia in the whole wheel's.
          parameter Real wheelRadius =      1;

          parameter Integer n = 5 "Number of rollers";
          parameter Real alpha = pi/n "Max angle of the half-sector";
          parameter Real R = 1 "Omni wheel outer radius";
          parameter Real d = platformDiameter "vehicle size";
          parameter Real R1 = R*cos(alpha) "Omni wheel inner radius";
          parameter Real om0_k = 2*pi/n*R/d;

          parameter Real om0 = om0_k * 1;
          parameter Real v0_abs = 0;
          parameter Real v0_dir = 0; //2*pi/3;

          parameter Real[3] r0 = {0, R, 0};
          parameter Real[4] q0 = {1, 0, 0, 0};

          parameter Real[3] v0 = v0_abs * {cos(v0_dir), 0, sin(v0_dir)};
          parameter Real[3] omega0 = {0, om0, 0};

          parameter Real[3] omega1_0 = omega0 + (-d)*om0/R * {0, 0, 1} + ((-1/R)*v0*{cos(pi/2 + 0*2*pi/3), 0, sin(pi/2 + 0*2*pi/3)}) * {0,0,1};
          parameter Real[3] omega2_0 = omega0 + (-d)*om0/R * {0, 0, 1} + ((-1/R)*v0*{cos(pi/2 + 1*2*pi/3), 0, sin(pi/2 + 1*2*pi/3)}) * {0,0,1};
          parameter Real[3] omega3_0 = omega0 + (-d)*om0/R * {0, 0, 1} + ((-1/R)*v0*{cos(pi/2 + 2*2*pi/3), 0, sin(pi/2 + 2*2*pi/3)}) * {0,0,1};

          // for plotting:
          Real _angle0(start = 0);
          Real _angle1(start = 0);
          Real _angle2(start = 0);
          Real _angle3(start = 0);
          Real _omega0(start = om0);
          Real _omega1(start = sqrt(omega1_0*omega1_0));
          Real _omega2(start = sqrt(omega2_0*omega2_0));
          Real _omega3(start = sqrt(omega3_0*omega3_0));
          Real _r1( start = r0[1]);
          Real _r3( start = r0[3]);
          Real _v1( start = v0[1]);
          Real _v3( start = v0[3]);

          Base Floor;

          FixedJoint Joint1(
            nA = {0, 0, 1},
            nB = {1, 0, 0},
            rA = {0, 0, 0},
            rB = {d, 0, 0});

          FixedJoint Joint2(
            nA = {0, 0, 1},
            nB = {-cos(pi/3), 0, -cos(pi/6)},
            rA = {0, 0, 0},
            rB = {-d*cos(pi/3), 0, -d*cos(pi/6)});

          FixedJoint Joint3(
            nA = {0, 0, 1},
            nB = {-cos(pi/3), 0, cos(pi/6)},
            rA = {0, 0, 0},
            rB = {-d*cos(pi/3), 0, d*cos(pi/6)});

          ThreePortsHeavyBody Platform(
            m = platformMass,
            I = diagonal({platformInertia for i in 1:3}),
            Gravity = {0, -1, 0},
            r(start = r0),
            v(start = v0),
            q(start = q0),
            omega(start = omega0));

          NRollersOmniWheel Wheel1(
            NRollers = n,
            R = wheelRadius,
            wheelMass = wheelMass,
            wheelInertia = wheelInertia,
            rollerFraction = rollerFraction,
            Gravity={0,-1,0},
            T0=[0,0,-1; 0,1,0; 1,0,0],
            r0=r0 + {d,0,0},
            v0=v0 + cross(omega0, {d,0,0}),
            q0=QMult(q0, {cos(pi/4),0,sin(pi/4),0}),
            omega0=omega1_0);

          NRollersOmniWheel Wheel2(
            NRollers = n,
            R = wheelRadius,
            wheelMass = wheelMass,
            wheelInertia = wheelInertia,
            rollerFraction = rollerFraction,
            Gravity={0,-1,0},
            T0=[-cos(pi/6),0,cos(pi/3); 0,1,0; -cos(pi/3),0,-cos(pi/6)],
            r0=r0 + (-d) * {cos(2*pi/3),0,sin(2*pi/3)},
            v0=v0 + cross(omega0, {-d*cos(pi/3),0,-d*cos(pi/6)}),
            q0=QMult(q0, {cos(7*pi/12),0,sin(7*pi/12),0}),
            omega0=omega2_0);

          NRollersOmniWheel Wheel3(
            NRollers = n,
            R = wheelRadius,
            wheelMass = wheelMass,
            wheelInertia = wheelInertia,
            rollerFraction = rollerFraction,
            Gravity={0,-1,0},
            T0=[cos(pi/6),0,cos(pi/3); 0,1,0; -cos(pi/3),0,cos(pi/6)],
            r0=r0 + (-d) * {cos(4*pi/3),0,sin(4*pi/3)},
            v0=v0 + cross(omega0, {-d*cos(pi/3),0,d*cos(pi/6)}),
            q0=QMult(q0, {cos(-pi/12),0,sin(-pi/12),0}),
            omega0=omega3_0);

        equation
          der(_angle0) = _omega0/(pi/180);
          der(_angle1) = _omega1/(pi/180);
          der(_angle2) = _omega2/(pi/180);
          der(_angle3) = _omega3/(pi/180);
          _omega0 = Platform.omega[2];
          _omega1 = sqrt(Wheel1.Wheel.omega[3]*Wheel1.Wheel.omega[3])*sign(Wheel1.Wheel.omega[3]);
          _omega2 = sqrt(Wheel2.Wheel.omega[3]*Wheel2.Wheel.omega[3])*sign(Wheel2.Wheel.omega[3]);
          _omega3 = sqrt(Wheel3.Wheel.omega[3]*Wheel3.Wheel.omega[3])*sign(Wheel3.Wheel.omega[3]);
          _r1 = Platform.r[1];
          _r3 = Platform.r[3];
          _v1 = Platform.v[1];
          _v3 = Platform.v[3];

          assert(noEvent(Platform.v[2]*Platform.v[2] < 10^(-precisionLevel_v)),  "Platform has vertical speed !!!");
          assert(noEvent(Platform.omega[1]*Platform.omega[1] + Platform.omega[3]*Platform.omega[3] < 10^(-precisionLevel_omega)),  "Platform.omega is not all [2] !!!");

          connect(Floor.OutPort, Wheel2.InPortK);
          connect(Wheel1.OutPortK,Joint1. InPortA);
          connect(Wheel2.OutPortK, Joint2.InPortA);
          connect(Wheel1.InPortF,Joint1. OutPortA);
          connect(Wheel2.InPortF, Joint2.OutPortA);
          connect(Platform.OutPort,Joint1. InPortB);
          connect(Platform.OutPort, Joint2.InPortB);
          connect(Joint2.OutPortB, Platform.InPort1);
          connect(Joint1.OutPortB, Platform.InPort);
          connect(Floor.OutPort, Wheel3.InPortK);
          connect(Wheel1.InPortK, Floor.OutPort);
          connect(Wheel3.OutPortK, Joint3.InPortA);
          connect(Joint3.InPortB, Platform.OutPort);
          connect(Wheel3.InPortF, Joint3.OutPortA);
          connect(Joint3.OutPortB, Platform.InPort2);

          annotation (experiment(NumberOfIntervals=500, Tolerance=1e-006), LogDefaultInitialConditions = true,
              __Dymola_experimentSetupOutput);
        end NRollersPointContactOmniWheelSetTest;

        model NRollersPointContactOmniWheelSetTestGeneral
          import Modelica.Constants.pi;

          parameter Real psi = 1e-1; //pi/6 "Angle of roller distortion (fixed axis turn)";

          //for debugging
          parameter Integer precisionLevel_v =     5;
          parameter Integer precisionLevel_omega = 6;

          // interface for comparing models
          parameter Real platformDiameter = 1;
          parameter Real platformMass = 10;
          parameter Real platformInertia = 10;
          parameter Real wheelMass = 1;
          parameter Real[3] wheelInertia = {1, 1, 1};
          parameter Real rollerFraction = 10^(-6); //0.3; //fraction of rollers' inertia in the whole wheel's.
          parameter Real wheelRadius =      1;

          parameter Integer n = 4 "Number of rollers";

          parameter Real alpha = pi/n "Max angle of the half-sector";
          parameter Real R = 1 "Omni wheel outer radius";
          parameter Real d = platformDiameter "vehicle size";
          parameter Real R1 = R*cos(alpha) "Omni wheel inner radius";
          parameter Real om0_k = 2*pi/n*R/d;

          parameter Real om0 = om0_k * 1;
          parameter Real v0_abs = 0;
          parameter Real v0_dir = 0; //2*pi/3;

          parameter Real[3] r0 = {0, R, 0};
          parameter Real[4] q0 = {1, 0, 0, 0};

          parameter Real[3] v0 = v0_abs * {cos(v0_dir), 0, sin(v0_dir)};
          parameter Real[3] omega0 = {0, om0, 0};

          parameter Real[3] omega1_0 = omega0 + (-d)*om0/R * {0, 0, 1} + ((-1/R)*v0*{cos(pi/2 + 0*2*pi/3), 0, sin(pi/2 + 0*2*pi/3)}) * {0,0,1};
          parameter Real[3] omega2_0 = omega0 + (-d)*om0/R * {0, 0, 1} + ((-1/R)*v0*{cos(pi/2 + 1*2*pi/3), 0, sin(pi/2 + 1*2*pi/3)}) * {0,0,1};
          parameter Real[3] omega3_0 = omega0 + (-d)*om0/R * {0, 0, 1} + ((-1/R)*v0*{cos(pi/2 + 2*2*pi/3), 0, sin(pi/2 + 2*2*pi/3)}) * {0,0,1};

          // for plotting:
          Real _angle0(start = 0);
          Real _angle1(start = 0);
          Real _angle2(start = 0);
          Real _angle3(start = 0);
          Real _omega0(start = om0);
          Real _omega1(start = sqrt(omega1_0*omega1_0));
          Real _omega2(start = sqrt(omega2_0*omega2_0));
          Real _omega3(start = sqrt(omega3_0*omega3_0));
          Real _r1( start = r0[1]);
          Real _r3( start = r0[3]);
          Real _v1( start = v0[1]);
          Real _v3( start = v0[3]);

          Base Floor;

          FixedJoint Joint1(
            nA = {0, 0, 1},
            nB = {1, 0, 0},
            rA = {0, 0, 0},
            rB = {d, 0, 0});

          FixedJoint Joint2(
            nA = {0, 0, 1},
            nB = {-cos(pi/3), 0, -cos(pi/6)},
            rA = {0, 0, 0},
            rB = {-d*cos(pi/3), 0, -d*cos(pi/6)});

          FixedJoint Joint3(
            nA = {0, 0, 1},
            nB = {-cos(pi/3), 0, cos(pi/6)},
            rA = {0, 0, 0},
            rB = {-d*cos(pi/3), 0, d*cos(pi/6)});

          ThreePortsHeavyBody Platform(
            m = platformMass,
            I = diagonal({platformInertia for i in 1:3}),
            Gravity = {0, -1, 0},
            r(start = r0),
            v(start = v0),
            q(start = q0),
            omega(start = omega0));

          NRollersOmniWheelGeneral Wheel1(
            NRollers = n,
            psi = psi,
            R = wheelRadius,
            wheelMass = wheelMass,
            wheelInertia = wheelInertia,
            rollerFraction = rollerFraction,
            Gravity={0,-1,0},
            T0=[0,0,-1; 0,1,0; 1,0,0],
            r0=r0 + {d,0,0},
            v0=v0 + cross(omega0, {d,0,0}),
            q0=QMult(q0, {cos(pi/4),0,sin(pi/4),0}),
            omega0=omega1_0);

          NRollersOmniWheelGeneral Wheel2(
            NRollers = n,
            psi = psi,
            R = wheelRadius,
            wheelMass = wheelMass,
            wheelInertia = wheelInertia,
            rollerFraction = rollerFraction,
            Gravity={0,-1,0},
            T0=[-cos(pi/6),0,cos(pi/3); 0,1,0; -cos(pi/3),0,-cos(pi/6)],
            r0=r0 + (-d) * {cos(2*pi/3),0,sin(2*pi/3)},
            v0=v0 + cross(omega0, {-d*cos(pi/3),0,-d*cos(pi/6)}),
            q0=QMult(q0, {cos(7*pi/12),0,sin(7*pi/12),0}),
            omega0=omega2_0);

          NRollersOmniWheelGeneral Wheel3(
            NRollers = n,
            psi = psi,
            R = wheelRadius,
            wheelMass = wheelMass,
            wheelInertia = wheelInertia,
            rollerFraction = rollerFraction,
            Gravity={0,-1,0},
            T0=[cos(pi/6),0,cos(pi/3); 0,1,0; -cos(pi/3),0,cos(pi/6)],
            r0=r0 + (-d) * {cos(4*pi/3),0,sin(4*pi/3)},
            v0=v0 + cross(omega0, {-d*cos(pi/3),0,d*cos(pi/6)}),
            q0=QMult(q0, {cos(-pi/12),0,sin(-pi/12),0}),
            omega0=omega3_0);

        equation
          der(_angle0) = _omega0/(pi/180);
          der(_angle1) = _omega1/(pi/180);
          der(_angle2) = _omega2/(pi/180);
          der(_angle3) = _omega3/(pi/180);
          _omega0 = Platform.omega[2];
          _omega1 = sqrt(Wheel1.Wheel.omega[3]*Wheel1.Wheel.omega[3])*sign(Wheel1.Wheel.omega[3]);
          _omega2 = sqrt(Wheel2.Wheel.omega[3]*Wheel2.Wheel.omega[3])*sign(Wheel2.Wheel.omega[3]);
          _omega3 = sqrt(Wheel3.Wheel.omega[3]*Wheel3.Wheel.omega[3])*sign(Wheel3.Wheel.omega[3]);
          _r1 = Platform.r[1];
          _r3 = Platform.r[3];
          _v1 = Platform.v[1];
          _v3 = Platform.v[3];

          assert(noEvent(Platform.v[2]*Platform.v[2] < 10^(-precisionLevel_v)),  "Platform has vertical speed !!!");
          assert(noEvent(Platform.omega[1]*Platform.omega[1] + Platform.omega[3]*Platform.omega[3] < 10^(-precisionLevel_omega)),  "Platform.omega is not all [2] !!!");

          connect(Floor.OutPort, Wheel2.InPortK);
          connect(Wheel1.OutPortK,Joint1. InPortA);
          connect(Wheel2.OutPortK, Joint2.InPortA);
          connect(Wheel1.InPortF,Joint1. OutPortA);
          connect(Wheel2.InPortF, Joint2.OutPortA);
          connect(Platform.OutPort,Joint1. InPortB);
          connect(Platform.OutPort, Joint2.InPortB);
          connect(Joint2.OutPortB, Platform.InPort1);
          connect(Joint1.OutPortB, Platform.InPort);
          connect(Floor.OutPort, Wheel3.InPortK);
          connect(Wheel1.InPortK, Floor.OutPort);
          connect(Wheel3.OutPortK, Joint3.InPortA);
          connect(Joint3.InPortB, Platform.OutPort);
          connect(Wheel3.InPortF, Joint3.OutPortA);
          connect(Joint3.OutPortB, Platform.InPort2);

          annotation (experiment(NumberOfIntervals=500, Tolerance=1e-006), LogDefaultInitialConditions = true,
              __Dymola_experimentSetupOutput);
        end NRollersPointContactOmniWheelSetTestGeneral;
      end PointContact;

      model NonHolonomicOmniPlatform
        "Modelica implementation of non-holonomic model by Borisov."
        import Modelica.Constants.pi;
       /*
   Moving axes Oe_1e_2, fixed in the platform.
 */

       // INITIAL VALUES -------------------------------------------------------------
       parameter Real[2] __0_v =  {1, 0};
       parameter Real __0_omega =   0;
       parameter Real __0_M =    0;
       // ----------------------------------------------------------------------------

       // ----------------------------------------------------------------------------
       parameter Real[2,2] E =   identity(2) "Identity matrix.";
       parameter Real[2,2] J =   [0, -1; 1, 0]
          "Skew matrix for vector crossing.";
       // ----------------------------------------------------------------------------

       // GEOMETRY -------------------------------------------------------------------
       parameter Real psi = 0 "Angle btw roller axis and wheel plane";
       parameter Integer N_WHEELS =   3 "Number of wheels.";
       parameter Real[N_WHEELS] wheelAngles = {2*pi/N_WHEELS*(i-1) for i in 1:N_WHEELS}
          "Directions to wheels from Platform center.";
       parameter Real platformDiameter =    1;
       parameter Real[2] rc =   {0, 0}
          "Masscenter radius-vector. RelLoc (Relative wrt moving).";
       parameter Real[N_WHEELS,2] r = {platformDiameter * {cos(wheelAngles[i]), sin(wheelAngles[i])} for i in 1:N_WHEELS}
          "Wheel radius-vectors. RelLoc.";
       parameter Real[N_WHEELS] delta = {pi/2 - psi for i in 1:N_WHEELS}
          "Pi/2 minus psi (psi is angle btw roller axis and wheel plane).";
       parameter Real[N_WHEELS,2] alpha = {{cos(delta[i]+wheelAngles[i]), sin(delta[i]+wheelAngles[i])} for i in 1:N_WHEELS}
          "Roller axes. RelLoc?";
       parameter Real[N_WHEELS] s = {sin(delta[i]) for i in 1:N_WHEELS}
          "sin(delta) (delta = pi/2 - ang btw axis and wheel).";
       parameter Real[N_WHEELS] h = {1 for i in 1:N_WHEELS} "Wheel radius.";

       Real[N_WHEELS, 2,2] alphaTensor = {{alpha[k,i]*alpha[k,j] for i in 1:2, j in 1:2} for k in 1:N_WHEELS}
          "alpha[i] (x) alpha[i] matrices (aT[k,i,j] = alpha[k,i]*alpha[k,j]).";
       // ----------------------------------------------------------------------------

       // INERTIA --------------------------------------------------------------------
       parameter Real[N_WHEELS] mi = {1 for i in 1:N_WHEELS} "Wheel masses.";
       parameter Real m0 =    10 "Platform mass.";
       parameter Real m =    m0 + sum(mi[i] for i in 1:N_WHEELS)
          "Total mass of the system.";

       parameter Real Ikr =    I + sum(Ii[i]/((s[i]^2)*h[i])* (J*r[i,:] * alpha[i,:])*(J*r[i,:] * alpha[i,:]) for i in 1:N_WHEELS)
          "Inertia, see other declarations. (I s kryshechkoj).";
       parameter Real I =    I0 + sum((mi[i]*r[i,:]*r[i,:] + Iiw[i]) for i in 1:N_WHEELS)
          "Total system inertia wrt point O.";
       parameter Real[N_WHEELS] Ii =  {1 for i in 1:N_WHEELS}
          "Wheel inertia wrt wheel axis. (I_i)";
       parameter Real[N_WHEELS] Iiw = {1/2 for i in 1:N_WHEELS}
          "Wheel inertia wrt wheel diameter (I with wave - s volnoj).";
       parameter Real I0 =    10 "Platform inertia.";
       // ----------------------------------------------------------------------------

       Real[2,2] Gamma
          "Roller axes' tensor multiplication matrix (see equations)";
       Real[2] v(   start = __0_v)
          "Velocity of point O, origin of moving axes. AbsLoc (Absolute wrt moving frame).";
       Real[2] a "Acceleration of point O, origin of moving axes. AbsLoc.";
       Real phi "Platform turn angle, grows counter-clockwise.";
       Real omega(   start = __0_omega)
          "Platform turn velocity, positive if counter-clockwise.";
       Real epsilon
          "Platform turn acceleration, positive if counter-clockwise.";
       Real[2] R "Hell of a rotational thing. See equations.";
       Real[N_WHEELS] mu "Normalized torques applied to wheel axes.";
       Real x "Masscenter X AbsGlo ?";
       Real y "Masscenter Y AbsGlo ?";
       Real[N_WHEELS] M "Torques applied to wheel axes.";

       Real[N_WHEELS] wheel_angle "Wheel turn angles";

      equation
       // MOTION EQUATIONS
       (Gamma + m*E)*a + m*epsilon*(J*rc + R) + m*omega*J*(v + omega*J*rc)
        = -sum(mu[i]*alpha[i,:] for i in 1:N_WHEELS);

       Ikr*epsilon + m*(J*rc + R)*a + m*omega*v*rc = -sum(mu[i]*((J*r[i,:])*alpha[i,:]) for i in 1:N_WHEELS);

       der(x) = v[1]*cos(phi) - v[2]*sin(phi);
       der(y) = v[1]*sin(phi) + v[2]*cos(phi);

       der(phi) = omega;

       der(v) = a;
       der(omega) = epsilon;
       // der(rc) = v; //?

       // HELPER
       Gamma = sum(Ii[i]/((s[i]*h[i])^2)*alphaTensor[i,:,:] for i in 1:N_WHEELS);
       R = 1/m*sum(Ii[i]/((s[i]*h[i])^2) * (J*r[i,:] * alpha[i,:])*alpha[i,:] for i in 1:N_WHEELS);
       mu = {M[i]/(s[i]*h[i]) for i in 1:N_WHEELS};

       for i in 1:N_WHEELS loop
         der(wheel_angle[i]) = -1/(s[i]*h[i]) * (v + omega*J*r[i,:])*alpha[i,:];
       end for;

       // PARAMETERS
       M = {__0_M for i in 1:N_WHEELS};

      end NonHolonomicOmniPlatform;

      model Comparison
        "Compares OmniWheelSet physical MBS model with Borisov non-holonomic."
        import Modelica.Constants.pi;

        parameter Real __0_v_x =             0;
        parameter Real __0_v_y =             0;
        parameter Real __0_omega =           1;
        parameter Integer __n_rollers =      4;
        parameter Real __rollerFraction =    1e-1;

        parameter Real platformDiameter = 1;
        parameter Real wheelMass =        1;
        parameter Real platformMass =     3.3;
        parameter Real wheelRadius =      1;
        parameter Real wheelAspectRatio = 1/5 "Thickness/Radius";
        parameter Real[3] wheelInertia =  wheelMass * (wheelRadius^2) * {1/2, 1/4 + 1/12*wheelAspectRatio^2, 1/4 + 1/12*wheelAspectRatio^2};
        //MBS prokidyvaet whIn stupize, a rolleram delaet individ.
        parameter Real platformInertia =  platformMass * ((platformDiameter/2)^2) / 2;

      //  parameter Real[3] rollerInertia = rollerMass * {1/2*(R-R1)^2, 3/12*(R-R1)^2 + 1/12*(2*R*sin(alpha))^2, 3/12*(R-R1)^2 + 1/12*(2*R*sin(alpha))^2};

        //should duplicate same from Wheel class:
        parameter Real rollerMass = wheelMass*__rollerFraction/__n_rollers;
      //  parameter Real alpha = pi/__n_rollers "Max angle of the half-sector";
      //  parameter Real[__n_rollers,4] roller_q0s_local = {{cos(-alpha*(i-1)), 0, 0, -1 * sin(-alpha*(i-1))} for i in 1:__n_rollers};
      //  parameter Real[__n_rollers,4] roller_q0s =       {QMult1({1,0,0,0}, roller_q0s_local[i,:]) for i in 1:__n_rollers};
        parameter Real[3] rollerInertia = rollerMass * {1/2, 1, 1};
      //  parameter Real R1 = wheelRadius*cos(alpha) "Omni wheel inner radius";
      //  parameter Real[__n_rollers,3] rollerDirs = {{cos(-pi/2 + 2*alpha*(i-1)), sin(-pi/2 + 2*alpha*(i-1)), 0} for i in 1:__n_rollers};
      //  parameter Real[__n_rollers,3,3] roller_T0s = {QToT(roller_q0s[i,:]) for i in 1:__n_rollers};
      //  parameter Real[__n_rollers] x = {rollerDirs[i,1]*R1 for i in 1:__n_rollers}
      //    "Roller x_0";
      //  parameter Real[__n_rollers] y = {rollerDirs[i,2]*R1 for i in 1:__n_rollers}
      //    "Roller y_0";
      //  parameter Real[__n_rollers] z = {rollerDirs[i,3]*R1 for i in 1:__n_rollers}
      //    "Roller z_0";
      //  parameter Real[3,3] nhlTotalWheelInertia = diagonal(wheelInertia) + sum(( transpose(roller_T0s[i,:,:])*diagonal(rollerInertia)*roller_T0s[i,:,:] +
      //                                                                            rollerMass * [ x[i]^2,                     -(x[i]^2+y[i]^2+x[i]*y[i]), -(x[i]^2+z[i]^2+x[i]*z[i]);
      //                                                                                           -(x[i]^2+y[i]^2+x[i]*y[i]), y[i]^2,                     -(y[i]^2+z[i]^2+y[i]*z[i]);
      //                                                                                           -(x[i]^2+z[i]^2+x[i]*z[i]), -(y[i]^2+z[i]^2+y[i]*z[i]), z[i]^2])                        for i in 1:__n_rollers);
        parameter Real nhlTotalWheelAxisInertia = wheelInertia[1] + 3*((wheelRadius*cos(pi/__n_rollers))^2)*rollerInertia[1];

        Real nhl_x;
        Real nhl_y;
        Real nhl_angle;
        Real[3] nhl_wheelAngles;

        Real mbs_x;
        Real mbs_y;
        Real mbs_angle;
        Real[3] mbs_wheelAngles;
        Real[3] mbs_vt;
        Real mbs_platform_r2;
        Real[3] mbs_omega;
        Real[3] mbs_rollerInContactNumber;
        Real[3,__n_rollers] mbs_contacts_h;
        Real[3] mbs_forceTsqrt;

        Real delta_x;
        Real delta_y;
        Real delta_angle;
        Real[3] delta_wheelAngles;

        NonHolonomicOmniPlatform nhl(
           __0_v =  {__0_v_x, __0_v_y},
           __0_omega = __0_omega,
           m0 = platformMass,
           mi = {wheelMass for i in 1:3},
           Ii = {nhlTotalWheelAxisInertia for i in 1:3},
           Iiw = {wheelInertia[2] for i in 1:3},
           I0 = platformInertia);
      //     Ii = {wheelInertia[1] for i in 1:3},

        PointContact.NRollersPointContactOmniWheelSetTest mbs(
           v0 = {__0_v_x, 0, __0_v_y},
           om0 = __0_omega,
           rollerFraction = __rollerFraction,
           n = __n_rollers,
           platformDiameter = platformDiameter,
           wheelRadius = wheelRadius,
           wheelMass = wheelMass,
           platformMass = platformMass,
           wheelInertia = {wheelInertia[2], wheelInertia[2], wheelInertia[1]},
           platformInertia = platformInertia);

      equation
        nhl_x     = nhl.x;
        nhl_y     = nhl.y;
        nhl_angle = nhl.phi*180/pi;
        nhl_wheelAngles = nhl.psi*180/pi;

        mbs_x     = mbs._r1;
        mbs_y     = mbs._r3;
        mbs_angle = mbs._angle0;
        mbs_wheelAngles = {mbs._angle1, mbs._angle2, mbs._angle3};
        mbs_vt = {mbs.Wheel1._vt, mbs.Wheel2._vt, mbs.Wheel3._vt};
        mbs_platform_r2 = mbs.Platform.r[2];
        mbs_omega = mbs.Platform.omega;
        mbs_rollerInContactNumber = {mbs.Wheel1._rollerInContactNumber, mbs.Wheel2._rollerInContactNumber, mbs.Wheel3._rollerInContactNumber};
        mbs_contacts_h = {{mbs.Wheel1.Contacts[i].h for i in 1:__n_rollers},
                          {mbs.Wheel2.Contacts[i].h for i in 1:__n_rollers},
                          {mbs.Wheel3.Contacts[i].h for i in 1:__n_rollers}};
        mbs_forceTsqrt = {sum(mbs.Wheel1.Contacts[i].ForceTsqrt*mbs.Wheel1.Contacts[i].isInContact for i in 1:__n_rollers),
                          sum(mbs.Wheel2.Contacts[i].ForceTsqrt*mbs.Wheel2.Contacts[i].isInContact for i in 1:__n_rollers),
                          sum(mbs.Wheel3.Contacts[i].ForceTsqrt*mbs.Wheel3.Contacts[i].isInContact for i in 1:__n_rollers)};

        delta_x     = mbs_x - nhl_x;
        delta_y     = mbs_y - nhl_y;
        delta_angle = mbs_angle - nhl_angle;
        delta_wheelAngles = mbs_wheelAngles - nhl_wheelAngles;

        annotation (experiment(StopTime=10, Tolerance=1e-007),
            __Dymola_experimentSetupOutput);
      end Comparison;

      model ComparisonMbsAndMbsGeneral
        "Compares OmniWheelSet physical MBS model with Borisov non-holonomic."
        import Modelica.Constants.pi;

        parameter Real __0_v_x =             0;
        parameter Real __0_v_y =             0;
        parameter Real __0_omega =           1;
        parameter Integer __n_rollers =      4;
        parameter Real __rollerFraction =    1e-1;

        parameter Real platformDiameter = 1;
        parameter Real wheelMass =        1;
        parameter Real platformMass =     3.3;
        parameter Real wheelRadius =      1;
        parameter Real wheelAspectRatio = 1/5 "Thickness/Radius";
        parameter Real[3] wheelInertia =  wheelMass * (wheelRadius^2) * {1/2, 1/4 + 1/12*wheelAspectRatio^2, 1/4 + 1/12*wheelAspectRatio^2};
        //MBS prokidyvaet whIn stupize, a rolleram delaet individ.
        parameter Real platformInertia =  platformMass * ((platformDiameter/2)^2) / 2;

      //  parameter Real[3] rollerInertia = rollerMass * {1/2*(R-R1)^2, 3/12*(R-R1)^2 + 1/12*(2*R*sin(alpha))^2, 3/12*(R-R1)^2 + 1/12*(2*R*sin(alpha))^2};

        //should duplicate same from Wheel class:
        parameter Real rollerMass = wheelMass*__rollerFraction/__n_rollers;
      //  parameter Real alpha = pi/__n_rollers "Max angle of the half-sector";
      //  parameter Real[__n_rollers,4] roller_q0s_local = {{cos(-alpha*(i-1)), 0, 0, -1 * sin(-alpha*(i-1))} for i in 1:__n_rollers};
      //  parameter Real[__n_rollers,4] roller_q0s =       {QMult1({1,0,0,0}, roller_q0s_local[i,:]) for i in 1:__n_rollers};
        parameter Real[3] rollerInertia = rollerMass * {1/2, 1, 1};
      //  parameter Real R1 = wheelRadius*cos(alpha) "Omni wheel inner radius";
      //  parameter Real[__n_rollers,3] rollerDirs = {{cos(-pi/2 + 2*alpha*(i-1)), sin(-pi/2 + 2*alpha*(i-1)), 0} for i in 1:__n_rollers};
      //  parameter Real[__n_rollers,3,3] roller_T0s = {QToT(roller_q0s[i,:]) for i in 1:__n_rollers};
      //  parameter Real[__n_rollers] x = {rollerDirs[i,1]*R1 for i in 1:__n_rollers}
      //    "Roller x_0";
      //  parameter Real[__n_rollers] y = {rollerDirs[i,2]*R1 for i in 1:__n_rollers}
      //    "Roller y_0";
      //  parameter Real[__n_rollers] z = {rollerDirs[i,3]*R1 for i in 1:__n_rollers}
      //    "Roller z_0";
      //  parameter Real[3,3] nhlTotalWheelInertia = diagonal(wheelInertia) + sum(( transpose(roller_T0s[i,:,:])*diagonal(rollerInertia)*roller_T0s[i,:,:] +
      //                                                                            rollerMass * [ x[i]^2,                     -(x[i]^2+y[i]^2+x[i]*y[i]), -(x[i]^2+z[i]^2+x[i]*z[i]);
      //                                                                                           -(x[i]^2+y[i]^2+x[i]*y[i]), y[i]^2,                     -(y[i]^2+z[i]^2+y[i]*z[i]);
      //                                                                                           -(x[i]^2+z[i]^2+x[i]*z[i]), -(y[i]^2+z[i]^2+y[i]*z[i]), z[i]^2])                        for i in 1:__n_rollers);
        parameter Real nhlTotalWheelAxisInertia = wheelInertia[1] + 3*((wheelRadius*cos(pi/__n_rollers))^2)*rollerInertia[1];

        parameter Real _psi = pi/6;

        Real mbs_x;
        Real mbs_y;
        Real mbs_angle;
        Real[3] mbs_wheelAngles;

        Real mbsGeneral_x;
        Real mbsGeneral_y;
        Real mbsGeneral_angle;
        Real[3] mbsGeneral_wheelAngles;

        Real[3] mbs_vt;
        Real mbs_platform_r2;
        Real[3] mbs_omega;
        Real[3] mbs_rollerInContactNumber;
        Real[3,__n_rollers] mbs_contacts_h;
        Real[3] mbs_forceTsqrt;

        Real delta_x;
        Real delta_y;
        Real delta_angle;
        Real[3] delta_wheelAngles;

        PointContact.NRollersPointContactOmniWheelSetTest mbs(
           v0 = {__0_v_x, 0, __0_v_y},
           om0 = __0_omega,
           rollerFraction = __rollerFraction,
           n = __n_rollers,
           platformDiameter = platformDiameter,
           wheelRadius = wheelRadius,
           wheelMass = wheelMass,
           platformMass = platformMass,
           wheelInertia = {wheelInertia[2], wheelInertia[2], wheelInertia[1]},
           platformInertia = platformInertia);

        PointContact.NRollersPointContactOmniWheelSetTestGeneral mbsGeneral(
           v0 = {__0_v_x, 0, __0_v_y},
           om0 = __0_omega,
           rollerFraction = __rollerFraction,
           n = __n_rollers,
           psi = _psi,
           platformDiameter = platformDiameter,
           wheelRadius = wheelRadius,
           wheelMass = wheelMass,
           platformMass = platformMass,
           wheelInertia = {wheelInertia[2], wheelInertia[2], wheelInertia[1]},
           platformInertia = platformInertia);

      equation
        mbs_x     = mbs._r1;
        mbs_y     = mbs._r3;
        mbs_angle = mbs._angle0;
        mbs_wheelAngles = {mbs._angle1, mbs._angle2, mbs._angle3};

        mbsGeneral_x     = mbsGeneral._r1;
        mbsGeneral_y     = mbsGeneral._r3;
        mbsGeneral_angle = mbsGeneral._angle0;
        mbsGeneral_wheelAngles = {mbsGeneral._angle1, mbsGeneral._angle2, mbsGeneral._angle3};

        mbs_vt = {mbs.Wheel1._vt, mbs.Wheel2._vt, mbs.Wheel3._vt};
        mbs_platform_r2 = mbs.Platform.r[2];
        mbs_omega = mbs.Platform.omega;
        mbs_rollerInContactNumber = {mbs.Wheel1._rollerInContactNumber, mbs.Wheel2._rollerInContactNumber, mbs.Wheel3._rollerInContactNumber};
        mbs_contacts_h = {{mbs.Wheel1.Contacts[i].h for i in 1:__n_rollers},
                          {mbs.Wheel2.Contacts[i].h for i in 1:__n_rollers},
                          {mbs.Wheel3.Contacts[i].h for i in 1:__n_rollers}};
        mbs_forceTsqrt = {sum(mbs.Wheel1.Contacts[i].ForceTsqrt*mbs.Wheel1.Contacts[i].isInContact for i in 1:__n_rollers),
                          sum(mbs.Wheel2.Contacts[i].ForceTsqrt*mbs.Wheel2.Contacts[i].isInContact for i in 1:__n_rollers),
                          sum(mbs.Wheel3.Contacts[i].ForceTsqrt*mbs.Wheel3.Contacts[i].isInContact for i in 1:__n_rollers)};

        delta_x     = mbs_x - mbsGeneral_x;
        delta_y     = mbs_y - mbsGeneral_y;
        delta_angle = mbs_angle - mbsGeneral_angle;
        delta_wheelAngles = mbs_wheelAngles - mbsGeneral_wheelAngles;

        annotation (experiment(StopTime=10, Tolerance=1e-007),
            __Dymola_experimentSetupOutput);
      end ComparisonMbsAndMbsGeneral;

      model ComparisonBorisovAndMbsGeneral
        "Compares OmniWheelSet physical MBS model with Borisov non-holonomic."
        import Modelica.Constants.pi;

        parameter Real __0_v_x =             0;
        parameter Real __0_v_y =             0;
        parameter Real __0_omega =           1;
        parameter Integer __n_rollers =      4;
        parameter Real __rollerFraction =    1e-1;

        parameter Real platformDiameter = 1;
        parameter Real wheelMass =        1;
        parameter Real platformMass =     3.3;
        parameter Real wheelRadius =      1;
        parameter Real wheelAspectRatio = 1/5 "Thickness/Radius";
        parameter Real[3] wheelInertia =  wheelMass * (wheelRadius^2) * {1/2, 1/4 + 1/12*wheelAspectRatio^2, 1/4 + 1/12*wheelAspectRatio^2};
        //MBS prokidyvaet whIn stupize, a rolleram delaet individ.
        parameter Real platformInertia =  platformMass * ((platformDiameter/2)^2) / 2;

      //  parameter Real[3] rollerInertia = rollerMass * {1/2*(R-R1)^2, 3/12*(R-R1)^2 + 1/12*(2*R*sin(alpha))^2, 3/12*(R-R1)^2 + 1/12*(2*R*sin(alpha))^2};

        //should duplicate same from Wheel class:
        parameter Real rollerMass = wheelMass*__rollerFraction/__n_rollers;
      //  parameter Real alpha = pi/__n_rollers "Max angle of the half-sector";
      //  parameter Real[__n_rollers,4] roller_q0s_local = {{cos(-alpha*(i-1)), 0, 0, -1 * sin(-alpha*(i-1))} for i in 1:__n_rollers};
      //  parameter Real[__n_rollers,4] roller_q0s =       {QMult1({1,0,0,0}, roller_q0s_local[i,:]) for i in 1:__n_rollers};
        parameter Real[3] rollerInertia = rollerMass * {1/2, 1, 1};
      //  parameter Real R1 = wheelRadius*cos(alpha) "Omni wheel inner radius";
      //  parameter Real[__n_rollers,3] rollerDirs = {{cos(-pi/2 + 2*alpha*(i-1)), sin(-pi/2 + 2*alpha*(i-1)), 0} for i in 1:__n_rollers};
      //  parameter Real[__n_rollers,3,3] roller_T0s = {QToT(roller_q0s[i,:]) for i in 1:__n_rollers};
      //  parameter Real[__n_rollers] x = {rollerDirs[i,1]*R1 for i in 1:__n_rollers}
      //    "Roller x_0";
      //  parameter Real[__n_rollers] y = {rollerDirs[i,2]*R1 for i in 1:__n_rollers}
      //    "Roller y_0";
      //  parameter Real[__n_rollers] z = {rollerDirs[i,3]*R1 for i in 1:__n_rollers}
      //    "Roller z_0";
      //  parameter Real[3,3] nhlTotalWheelInertia = diagonal(wheelInertia) + sum(( transpose(roller_T0s[i,:,:])*diagonal(rollerInertia)*roller_T0s[i,:,:] +
      //                                                                            rollerMass * [ x[i]^2,                     -(x[i]^2+y[i]^2+x[i]*y[i]), -(x[i]^2+z[i]^2+x[i]*z[i]);
      //                                                                                           -(x[i]^2+y[i]^2+x[i]*y[i]), y[i]^2,                     -(y[i]^2+z[i]^2+y[i]*z[i]);
      //                                                                                           -(x[i]^2+z[i]^2+x[i]*z[i]), -(y[i]^2+z[i]^2+y[i]*z[i]), z[i]^2])                        for i in 1:__n_rollers);
        parameter Real nhlTotalWheelAxisInertia = wheelInertia[1] + 3*((wheelRadius*cos(pi/__n_rollers))^2)*rollerInertia[1];

        parameter Real _psi = pi/6;

        Real nhl_x;
        Real nhl_y;
        Real nhl_angle;
        Real[3] nhl_wheelAngles;

        Real mbs_x;
        Real mbs_y;
        Real mbs_angle;
        Real[3] mbs_wheelAngles;

        Real[3] mbs_vt;
        Real mbs_platform_r2;
        Real[3] mbs_omega;
        Real[3] mbs_rollerInContactNumber;
        Real[3,__n_rollers] mbs_contacts_h;
        Real[3] mbs_forceTsqrt;

        Real delta_x;
        Real delta_y;
        Real delta_angle;
        Real[3] delta_wheelAngles;

        NonHolonomicOmniPlatform nhl(
           psi = _psi,
           __0_v =  {__0_v_x, __0_v_y},
           __0_omega = __0_omega,
           m0 = platformMass,
           mi = {wheelMass for i in 1:3},
           Ii = {nhlTotalWheelAxisInertia for i in 1:3},
           Iiw = {wheelInertia[2] for i in 1:3},
           I0 = platformInertia);
      //     Ii = {wheelInertia[1] for i in 1:3},

        PointContact.NRollersPointContactOmniWheelSetTestGeneral mbs(
           psi = _psi,
           v0 = {__0_v_x, 0, __0_v_y},
           om0 = __0_omega,
           rollerFraction = __rollerFraction,
           n = __n_rollers,
           platformDiameter = platformDiameter,
           wheelRadius = wheelRadius,
           wheelMass = wheelMass,
           platformMass = platformMass,
           wheelInertia = {wheelInertia[2], wheelInertia[2], wheelInertia[1]},
           platformInertia = platformInertia);

      equation
        nhl_x     = nhl.x;
        nhl_y     = nhl.y;
        nhl_angle = nhl.phi*180/pi;
        nhl_wheelAngles = nhl.wheel_angle*180/pi;

        mbs_x     = mbs._r1;
        mbs_y     = mbs._r3;
        mbs_angle = mbs._angle0;
        mbs_wheelAngles = {mbs._angle1, mbs._angle2, mbs._angle3};
        mbs_vt = {mbs.Wheel1._vt, mbs.Wheel2._vt, mbs.Wheel3._vt};
        mbs_platform_r2 = mbs.Platform.r[2];
        mbs_omega = mbs.Platform.omega;
        mbs_rollerInContactNumber = {mbs.Wheel1._rollerInContactNumber, mbs.Wheel2._rollerInContactNumber, mbs.Wheel3._rollerInContactNumber};
        mbs_contacts_h = {{mbs.Wheel1.Contacts[i].h for i in 1:__n_rollers},
                          {mbs.Wheel2.Contacts[i].h for i in 1:__n_rollers},
                          {mbs.Wheel3.Contacts[i].h for i in 1:__n_rollers}};
        mbs_forceTsqrt = {sum(mbs.Wheel1.Contacts[i].ForceTsqrt*mbs.Wheel1.Contacts[i].isInContact for i in 1:__n_rollers),
                          sum(mbs.Wheel2.Contacts[i].ForceTsqrt*mbs.Wheel2.Contacts[i].isInContact for i in 1:__n_rollers),
                          sum(mbs.Wheel3.Contacts[i].ForceTsqrt*mbs.Wheel3.Contacts[i].isInContact for i in 1:__n_rollers)};

        delta_x     = mbs_x - nhl_x;
        delta_y     = mbs_y - nhl_y;
        delta_angle = mbs_angle - nhl_angle;
        delta_wheelAngles = mbs_wheelAngles - nhl_wheelAngles;

        annotation (experiment(StopTime=10, Tolerance=1e-007),
            __Dymola_experimentSetupOutput);
      end ComparisonBorisovAndMbsGeneral;

      model ComparisonMbsGeneralAndBorisov
        "Compares OmniWheelSet physical MBS model with Borisov non-holonomic."
        import Modelica.Constants.pi;

        parameter Real __0_v_x =             0;
        parameter Real __0_v_y =             0;
        parameter Real __0_omega =           1;
        parameter Integer __n_rollers =      4;
        parameter Real __rollerFraction =    1e-1;

        parameter Real platformDiameter = 1;
        parameter Real wheelMass =        1;
        parameter Real platformMass =     3.3;
        parameter Real wheelRadius =      1;
        parameter Real wheelAspectRatio = 1/5 "Thickness/Radius";
        parameter Real[3] wheelInertia =  wheelMass * (wheelRadius^2) * {1/2, 1/4 + 1/12*wheelAspectRatio^2, 1/4 + 1/12*wheelAspectRatio^2};
        //MBS prokidyvaet whIn stupize, a rolleram delaet individ.
        parameter Real platformInertia =  platformMass * ((platformDiameter/2)^2) / 2;

      //  parameter Real[3] rollerInertia = rollerMass * {1/2*(R-R1)^2, 3/12*(R-R1)^2 + 1/12*(2*R*sin(alpha))^2, 3/12*(R-R1)^2 + 1/12*(2*R*sin(alpha))^2};

        //should duplicate same from Wheel class:
        parameter Real rollerMass = wheelMass*__rollerFraction/__n_rollers;
      //  parameter Real alpha = pi/__n_rollers "Max angle of the half-sector";
      //  parameter Real[__n_rollers,4] roller_q0s_local = {{cos(-alpha*(i-1)), 0, 0, -1 * sin(-alpha*(i-1))} for i in 1:__n_rollers};
      //  parameter Real[__n_rollers,4] roller_q0s =       {QMult1({1,0,0,0}, roller_q0s_local[i,:]) for i in 1:__n_rollers};
        parameter Real[3] rollerInertia = rollerMass * {1/2, 1, 1};
      //  parameter Real R1 = wheelRadius*cos(alpha) "Omni wheel inner radius";
      //  parameter Real[__n_rollers,3] rollerDirs = {{cos(-pi/2 + 2*alpha*(i-1)), sin(-pi/2 + 2*alpha*(i-1)), 0} for i in 1:__n_rollers};
      //  parameter Real[__n_rollers,3,3] roller_T0s = {QToT(roller_q0s[i,:]) for i in 1:__n_rollers};
      //  parameter Real[__n_rollers] x = {rollerDirs[i,1]*R1 for i in 1:__n_rollers}
      //    "Roller x_0";
      //  parameter Real[__n_rollers] y = {rollerDirs[i,2]*R1 for i in 1:__n_rollers}
      //    "Roller y_0";
      //  parameter Real[__n_rollers] z = {rollerDirs[i,3]*R1 for i in 1:__n_rollers}
      //    "Roller z_0";
      //  parameter Real[3,3] nhlTotalWheelInertia = diagonal(wheelInertia) + sum(( transpose(roller_T0s[i,:,:])*diagonal(rollerInertia)*roller_T0s[i,:,:] +
      //                                                                            rollerMass * [ x[i]^2,                     -(x[i]^2+y[i]^2+x[i]*y[i]), -(x[i]^2+z[i]^2+x[i]*z[i]);
      //                                                                                           -(x[i]^2+y[i]^2+x[i]*y[i]), y[i]^2,                     -(y[i]^2+z[i]^2+y[i]*z[i]);
      //                                                                                           -(x[i]^2+z[i]^2+x[i]*z[i]), -(y[i]^2+z[i]^2+y[i]*z[i]), z[i]^2])                        for i in 1:__n_rollers);
        parameter Real nhlTotalWheelAxisInertia = wheelInertia[1] + 3*((wheelRadius*cos(pi/__n_rollers))^2)*rollerInertia[1];

        parameter Real _psi = 0;

        NonHolonomicOmniPlatform nhl(
           psi = _psi,
           __0_v =  {__0_v_x, __0_v_y},
           __0_omega = __0_omega,
           m0 = platformMass,
           mi = {wheelMass for i in 1:3},
           Ii = {nhlTotalWheelAxisInertia for i in 1:3},
           Iiw = {wheelInertia[2] for i in 1:3},
           I0 = platformInertia);
      //     Ii = {wheelInertia[1] for i in 1:3},

        PointContact.NRollersPointContactOmniWheelSetTestGeneral mbs(
           v0 = {__0_v_x, 0, __0_v_y},
           om0 = __0_omega,
           rollerFraction = __rollerFraction,
           n = __n_rollers,
           psi = _psi,
           platformDiameter = platformDiameter,
           wheelRadius = wheelRadius,
           wheelMass = wheelMass,
           platformMass = platformMass,
           wheelInertia = {wheelInertia[2], wheelInertia[2], wheelInertia[1]},
           platformInertia = platformInertia);

        annotation (experiment(StopTime=10, Tolerance=1e-007),
            __Dymola_experimentSetupOutput);
      end ComparisonMbsGeneralAndBorisov;
    end OmniVehicle;
  end Examples;

  annotation (uses(Modelica(version="1.6")));

  model NPortsHeavyBody
    extends RigidBody;

    parameter SI.Acceleration[3] Gravity;
    parameter Integer NPorts = 1;
    WrenchPort[NPorts] InPorts;
  equation
    F = m*Gravity + sum(InPorts[i].F for i in 1:NPorts);
    M = sum(InPorts[i].M + cross(InPorts[i].P - r, InPorts[i].F) for i in 1:NPorts);

  end NPortsHeavyBody;

  function QMult1 "No protected - for array construction"
    input Real q1[4];
    input Real q2[4];
    output Real q3[4];
  algorithm
    q3 := {q1[1]*q2[1] - {q1[2],q1[3],q1[4]}*{q2[2],q2[3],q2[4]},
           (q1[1]*{q2[2],q2[3],q2[4]} + q2[1]*{q1[2],q1[3],q1[4]} + cross({q1[2],q1[3],q1[4]},{q2[2],q2[3],q2[4]}))[1],
           (q1[1]*{q2[2],q2[3],q2[4]} + q2[1]*{q1[2],q1[3],q1[4]} + cross({q1[2],q1[3],q1[4]}, {q2[2],q2[3],q2[4]}))[2],
           (q1[1]*{q2[2],q2[3],q2[4]} + q2[1]*{q1[2],q1[3],q1[4]} + cross({q1[2],q1[3],q1[4]}, {q2[2],q2[3],q2[4]}))[3]};
    annotation(Inline=true);
  end QMult1;

  model ForceBase
                  extends Base;
    WrenchPort InPort;
  equation
    InPort.P = zeros(3);
    InPort.F = zeros(3);
    InPort.M = zeros(3);
  end ForceBase;

  function QToT "Converts quaternion to Rotation matrix."
    input Real[4] q;
    output Real[3,3] T;

  algorithm
    T := [q[1]^2 + q[2]^2 - q[3]^2 - q[4]^2, 2*(q[2]*q[3] - q[1]*q[4]), 2*(q[2]*
      q[4] + q[1]*q[3]); 2*(q[1]*q[4] + q[2]*q[3]), q[1]^2 - q[2]^2 + q[3]^2 -
      q[4]^2, 2*(q[3]*q[4] - q[1]*q[2]); 2*(q[2]*q[4] - q[1]*q[3]), 2*(q[1]*q[2]
       + q[3]*q[4]), q[1]^2 - q[2]^2 - q[3]^2 + q[4]^2]/(q*q);
    annotation(Inline=true);
  end QToT;
end ThreeD_MBS_Dynamics;
