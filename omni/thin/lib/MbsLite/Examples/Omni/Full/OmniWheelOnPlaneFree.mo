within MbsLite.Examples.Omni.Full;

model OmniWheelOnPlaneFree
  import MbsLite.Examples.Omni.Contact.OmniWheelOnPlane;

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

initial algorithm
  AssertInitialized (name, q0,      "q0");
  AssertInitialized (name, r0,      "r0");
  AssertInitialized (name, Gravity, "Gravity");

equation

  wheelOnPlane.wheel.InPortF.P = wheelOnPlane.wheel.OutPortK.r;
  wheelOnPlane.wheel.InPortF.F = zeros(3);
  wheelOnPlane.wheel.InPortF.M = zeros(3);

end OmniWheelOnPlaneFree;

