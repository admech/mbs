within MBS;

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
