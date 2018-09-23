within MBS.Examples.OmniVehicle.PointContact;

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
