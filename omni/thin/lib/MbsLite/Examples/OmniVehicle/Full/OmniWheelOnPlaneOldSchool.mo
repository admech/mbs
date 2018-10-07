within MbsLite.Examples.OmniVehicle.Full;

model OmniWheelOnPlaneOldSchool

  import MbsLite.Examples.OmniVehicle.OmniWheel;
  import MbsLite.Examples.OmniVehicle.Contact.PlaneContactOldSchool;

  parameter String  name   = "OmniWheelAtRest";
  parameter Boolean strict = false;

  parameter Real[3]  Gravity = fill(inf, 3);
  parameter Integer  nActual = -Integer_inf;
  parameter Real[3]  r0      = fill(inf, 3);
  parameter Real[4]  q0      = fill(inf, 3);
  parameter Params   params;
  parameter Initials initials;

  Base base;

  OmniWheel wheel
    ( final name     = "wheel"
    , final nActual  = nActual
    , final Gravity  = Gravity
    , final r0       = params.wheelRadius * vertical
    , final q0       = q0
    , final params   = params
    , final initials = initials
    );

  PlaneContactOldSchool[nActual] contacts
    ( final name = { "contacts[" + String(i) + "]" for i in 1 : nActual }
    , each final params                        = params
    , each final frictionCoeff                 = 1e-1
    , each final viscousFrictionVelocityBound  = 1e-6
    );
  
  // for visualization only! likely to spoil index reduction
  Integer indexOfRollerInContact;
  Real    contactPointVelocity;

initial algorithm
  AssertInitialized (name, q0,      "q0");
  AssertInitialized (name, r0,      "r0");
  AssertInitialized (name, Gravity, "Gravity");

equation
  
  if CompareReal(time, floor(time)) then
    print("Current simulation time: " + String(time));
  end if;
 
  wheel.InPortF.P = wheel.OutPortK.r;
  wheel.InPortF.F = zeros(3);
  wheel.InPortF.M = zeros(3);
  for i in 1 : nActual loop
    connect(contacts[i].InPortA,    base.OutPort);
    connect(contacts[i].InPortB,    wheel.Rollers[i].OutPort);
    connect(contacts[i].OutPortB,   wheel.Rollers[i].InPorts[1]);
  end for;
  indexOfRollerInContact = Argmin
    ( { (if contacts[i].isInContact then -1 else 0)
      for i in 1 : nActual
      }
    );
  contactPointVelocity = sum
    ( { sign
          ( normalize(contacts[i].contactPointVelocity)
          * contacts[i].contactPointVelocity
          )
        * contacts[i].contactPointVelocityNorm
      for i in 1 : nActual
      }
    );

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

end OmniWheelOnPlaneOldSchool;

