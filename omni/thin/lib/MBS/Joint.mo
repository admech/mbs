within MBS;

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
