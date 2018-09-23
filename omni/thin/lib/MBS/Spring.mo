within MBS;

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
