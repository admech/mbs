within MbsLite.Examples.OmniVehicle.Full;

model OmniWheelVertical

  parameter String  name   = "OmniWheelOnPlaneFree";

  parameter Real[3]  Gravity = fill(inf, 3);
  parameter Integer  nActual = -Integer_inf;
  parameter Real[3]  r0      = fill(inf, 3);
  parameter Real[4]  q0      = fill(inf, 4);
  parameter Params   params;
  parameter Initials initials;

  OmniWheelOnPlane wheelOnPlane
    ( name     = "OmniWheelOnPlane"
    , nActual  = nActual
    , Gravity  = Gravity
    , r0       = r0
    , q0       = q0
    , params   = params
    , initials = initials
    );

  Real torque;
  Real epsilonNaklon;
  Real omegaNaklon;

initial algorithm
  AssertInitialized (name, q0,      "q0");
  AssertInitialized (name, r0,      "r0");
  AssertInitialized (name, Gravity, "Gravity");

equation
 
  omegaNaklon = (wheelOnPlane.outPort.T * forward) * wheelOnPlane.outPort.omega;
  der(omegaNaklon) = epsilonNaklon;
  epsilonNaklon = 0;

  wheelOnPlane.InPort.P = wheelOnPlane.outPort.r;
  wheelOnPlane.InPort.F = zeros(3);
  wheelOnPlane.InPort.M = (wheelOnPlane.outPort.T * forward) * torque;

end OmniWheelVertical;

