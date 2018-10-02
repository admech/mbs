within MbsLite.Test.Constraints;

model JointTest

  FixedJoint joint
    ( name = "joint"
    , nA = { 0, 0, 1 }
    , nB = { 0, 0, 1 }
    , rA = { 0, 0, 0 }
    , rB = { 0, 0, 0 }
    );

  NPortsHeavyBody a
    ( name = "body A"
    , N = 1
    , Gravity = { 0, -1, 0 }
    , m = 1
    , I = identity(3)
    , r(start = { 0, 0, 0 })
    , v(start = { 0, 0, 0 })
    , q(start = QRot(0, { 0, 0, 1 }))
    , omega(start = { 0, 0, 0 })
    );

  NPortsHeavyBody b
    ( name = "body B"
    , N = 2
    , Gravity = zeros(3)
    , m = 1
    , I = identity(3)
    , r(start = { 0, 0, 0 })
    , v(start = { 0, 0, 0 })
    , q(start = QRot(0, { 0, 0, 1 }))
    , omega(start = { 0, 0, 0 })
    );

constant Real[3] expectedF = { 0, -0.5, 0 };
constant Real[3] expectedM = { 0,  0.5, 0 };
initial equation
  for i in 1 : 3 loop
    Assert
      ( CompareReal(expectedF[i], a.a[i])
      , "Body A acceleration should have been " + StringA(expectedF) + ", but was: " + StringA(a.a)
      );
    Assert
      ( CompareReal(expectedF[i], b.a[i])
      , "Body B acceleration should have been " + StringA(expectedF) + ", but was: " + StringA(b.a)
      );
    Assert
      ( CompareReal(expectedF[i], a.F[i])
      , "Body A force should have been " + StringA(expectedF) + ", but was: " + StringA(a.F)
      );
    Assert
      ( CompareReal(expectedF[i], b.F[i])
      , "Body B force should have been " + StringA(expectedF) + ", but was: " + StringA(b.F)
      );
    Assert
      ( CompareReal(expectedM[i], a.epsilon[i])
      , "Body A angular acceleration should have been " + StringA(expectedM) + ", but was: " + StringA(a.epsilon)
      );
    Assert
      ( CompareReal(expectedM[i], b.epsilon[i])
      , "Body B angular acceleration should have been " + StringA(expectedM) + ", but was: " + StringA(b.epsilon)
      );
    Assert
      ( CompareReal(expectedM[i], a.M[i])
      , "Body A torque should have been " + StringA(expectedM) + ", but was: " + StringA(a.M)
      );
    Assert
      ( CompareReal(expectedM[i], b.M[i])
      , "Body B torque should have been " + StringA(expectedM) + ", but was: " + StringA(b.M)
      );
  end for;
  terminate("passed");

equation

  connect(a.OutPort, joint.InPortA);
  connect(b.OutPort, joint.InPortB);

  connect(a.InPorts[1], joint.OutPortA);
  connect(b.InPorts[1], joint.OutPortB);

  b.InPorts[2].P = { 0, 0, 0 };
  b.InPorts[2].F = { 0, 0, 0 };
  b.InPorts[2].M = { 0, 1, 0 };

  annotation(experiment
    ( StopTime = 0.1
    , NumberOfIntervals = 100
    , Tolerance = 1e-9
    , Algorithm = "Dassl"
    ));

  when time > 0 then
    Assert
      ( false
      , "should have terminated after checking the solution to the initial problem"
      );
  end when;

end JointTest;
