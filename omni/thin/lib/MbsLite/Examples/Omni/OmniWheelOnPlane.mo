within MbsLite.Examples.Omni;

model OmniWheelOnPlane

  parameter String  name   = "OmniWheelOnPlane";
  parameter Boolean strict = false;

  parameter Real[3]  Gravity = fill(inf, 3);
  parameter Integer  nActual = -Integer_inf;
  parameter Real[3]  r0      = fill(inf, 3);
  parameter Real[4]  q0      = fill(inf, 4);
  parameter Params           params;
  parameter Initials         initials;
  parameter FrictionParams   frictionParams;

  parameter Real firstRollerAxialOmega0 = 0;
  parameter Real wheelAxialOmega0       = 0;

  Base base;

  OmniWheel wheel
    ( name     = name + ".wheel"
    , nActual  = nActual
    , Gravity  = Gravity
    , r0       = r0
    , q0       = q0
    , params   = params
    , initials = initials
    , frictionParams = frictionParams
    );

  PlaneContact[nActual] contacts
    ( name = { name + ".contacts[" + String(i) + "]" for i in 1 : nActual }
    , each params                        = params
    , isInContactInitially               =
        { (if i == 1 then true else false) for i in 1 : nActual }
    , each frictionParams                = frictionParams
    );

  // for visualization only! likely to spoil index reduction
  Integer indexOfRollerInContact;
  Real    contactPointVelocity;
  Real    contactPointNormalVelocity;
  Real    friction;
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
    ( { (if contacts[i].isInContactNormal then -1 else 0)
      for i in 1 : nActual
      }
    );
  // the norm of contact point velocity with sign -- note that it can become zero due to projection becoming zero
  contactPointVelocity = sum
    ( { contacts[i].contactPointVelocityNorm
        * sign
            ( (contacts[i].InPortB.T * userward)
            * contacts[i].contactPointVelocity
            )
      for i in 1 : nActual
      }
    );
  contactPointNormalVelocity = sum
    ( { contacts[i].contactPointVelocity[2]
      for i in 1 : nActual
      }
    );
  friction = sum
    ( { contacts[i].frictionNorm
        * sign
            ( (contacts[i].InPortB.T * userward)
            * contacts[i].friction
            )
      for i in 1 : nActual
      }
    );
  normalReaction = sum
    ( { (if contacts[i].isInContactNormal then contacts[i].normalReaction else 0)
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

/*
  when pre(indexOfRollerInContact) <> indexOfRollerInContact then
    reinit(wheel.Wheel.r[2], params.wheelRadius);
  end when;
*/

end OmniWheelOnPlane;

