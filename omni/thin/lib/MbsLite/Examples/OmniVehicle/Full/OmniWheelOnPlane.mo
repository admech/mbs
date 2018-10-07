within MbsLite.Examples.OmniVehicle.Full;

model OmniWheelOnPlane

  import MbsLite.Examples.OmniVehicle.OmniWheel;
  import MbsLite.Examples.OmniVehicle.Contact.PlaneContact;

  parameter String  name   = "OmniWheelAtRest";
  parameter Boolean strict = false;

  parameter Real[3]  Gravity = fill(inf, 3);
  parameter Integer  nActual = -Integer_inf;
  parameter Real[3]  r0      = fill(inf, 3);
  parameter Real[4]  q0      = fill(inf, 3);
  parameter Params   params;
  parameter Initials initials;

  OmniWheel wheel
    ( final name     = "wheel"
    , final nActual  = nActual
    , final Gravity  = Gravity
    , final r0       = params.wheelRadius * vertical
    , final q0       = q0
    , final params   = params
    , final initials = initials
    );

  PlaneContact contact
    ( final params   = params
    , final nActual  = nActual
    );

initial algorithm
  AssertInitialized (name, q0,      "q0");
  AssertInitialized (name, r0,      "r0");
  AssertInitialized (name, Gravity, "Gravity");

equation
 
  connect(contact.wheelOutPort,     wheel.OutPortK);
  connect(contact.wheelInPort,      wheel.InPortF);
  for i in 1 : nActual loop
    connect(contact.rollerOutPorts[i],   wheel.Rollers[i].OutPort);
    connect(contact.rollerInPorts[i],    wheel.Rollers[i].InPorts[1]);
  end for;

  if strict then
    when wheel.OutPortK.r[2] < R then
      assert(CompareReal(R, wheel.OutPortK.r[2]), "the wheel is falling down! (r[2] < R)");
    end when;
    when wheel.OutPortK.r[2] > R then
      assert(CompareReal(R, wheel.OutPortK.r[2]), "the wheel is flying! (r[2] > R)");
    end when;
    when wheel.OutPortK.v[2] < 0 then
      assert(CompareReal(0, wheel.OutPortK.v[2]), "the wheel is falling down! (v[2] < 0)");
    end when;
    when wheel.OutPortK.v[2] > 0 then
      assert(CompareReal(0, wheel.OutPortK.v[2]), "the wheel is flying! (v[2] > 0)");
    end when;
  end if;

end OmniWheelOnPlane;

