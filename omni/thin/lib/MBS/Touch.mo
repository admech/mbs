within MBS;

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
