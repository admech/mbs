within MbsLite.Examples.OmniVehicle.PointContact;

model OmniWheelOnPlane
  import MbsLite.Examples.OmniVehicle.Contact.PlaneContactOldSchool;

  parameter String  name   = "OmniWheelOnPlane";
  parameter Boolean strict = false;

  parameter Real[3]  Gravity = fill(inf, 3);
  parameter Integer  nActual = -Integer_inf;
  parameter Real[3]  r0      = fill(inf, 3);
  parameter Real[4]  q0      = fill(inf, 4);
  parameter Params   params;
  parameter Initials initials;

  Base base;

  OmniWheel wheel
    ( name     = name + ".wheel"
    , nActual  = nActual
    , Gravity  = Gravity
    , r0       = r0
    , q0       = q0
    , params   = params
    , initials = initials
    );

  PlaneContactOldSchool[nActual] contacts
    ( name = { name + ".contacts[" + String(i) + "]" for i in 1 : nActual }
    , each params                        = params
    , each frictionCoeff                 = 1e-1
    , each viscousFrictionVelocityBound  = 1e-6
    );

  // for visualization only! likely to spoil index reduction
  Integer indexOfRollerInContact;
  Real    contactPointVelocity;
  Real    friction;
  Real    normalVelocity;
  Real    normalReaction;

initial algorithm
  AssertInitializedI(name, nActual, "nActual");
  AssertInitialized (name, q0,      "q0");
  AssertInitialized (name, r0,      "r0");
  AssertInitialized (name, Gravity, "Gravity");

equation
  
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
  // FIXME: strange calculations here; wanted to show the norm of velocity with sign
  contactPointVelocity = sum
    ( { sign
          ( normalize(contacts[i].contactPointVelocity)
          * contacts[i].contactPointVelocity
          )
        * contacts[i].contactPointVelocityNorm
      for i in 1 : nActual
      }
    );
  friction = sum
    ( { sign
          ( normalize(contacts[i].friction)
          * contacts[i].friction
          )
        * norm(contacts[i].friction)
      for i in 1 : nActual
      }
    );
  normalVelocity = sum
    ( { contacts[i].normalVelocity
      for i in 1 : nActual
      }
    );
  normalReaction = sum
    ( { (if contacts[i].isInContact then contacts[i].normalReaction else 0)
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

end OmniWheelOnPlane;

