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
