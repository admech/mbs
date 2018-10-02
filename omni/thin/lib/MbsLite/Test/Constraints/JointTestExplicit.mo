within MbsLite.Test.Constraints;

model JointTestExplicit
  "This model is over-determined and does not simulate"

  FixedJoint joint
    ( name = "joint"
    , nA = { 1, 0, 0 }
    , nB = { 1, 0, 0 }
    , rA = { 0, 0, 0 }
    , rB = { 0, 0, 0 }
    );

  // specified
  Real[3]   rA        (start = { 0, 0, 0 });
  Real[3]   vA        (start = { 0, 0, 0 });
  Real[4]   qA        (start = QRot(0, { 0, 1, 0 }));
  Real[3]   omegaA    (start = { 0, 0, 0 }); // in local coords
  // calculated via r, v, q, omega
  Real[3,3] TA;
  Real[3]   FA;
  Real[3]   MA; // in global coords
  // calculated via F, M
  Real[3]   aA;
  Real[3]   epsilonA;

  // specified
  Real[3]   rB        (start = { 0, 0, 0 });
  Real[3]   vB        (start = { 0, 0, 0 });
  Real[4]   qB        (start = QRot(0, { 0, 1, 0 }));
  Real[3]   omegaB    (start = { 0, 0, 0 }); // in local coords
  // calculated via r, v, q, omega
  Real[3,3] TB;
  Real[3]   FB;
  Real[3]   MB; // in global coords
  // calculated via F, M
  Real[3]   aB;
  Real[3]   epsilonB;

equation

  TA = QToT(qA);
  TB = QToT(qB);

  // describe the system dynamics

  der(rA) = vA;
  der(vA) = aA;
  aA = FA;
  der(qA) = 0.5 * QMult(qA, { 0, omegaA[1], omegaA[2], omegaA[3] });
  der(omegaA) = epsilonA;
  epsilonA = transpose(TA) * MA;

  der(rB) = vB;
  der(vB) = aB;
  aB = FB;
  der(qB) = 0.5 * QMult(qB, { 0, omegaB[1], omegaB[2], omegaB[3] });
  der(omegaB) = epsilonB;
  epsilonB = transpose(TB) * MB;

  // pass the specified information into the constraint

  joint.InPortA.r       = rA;
  joint.InPortA.v       = vA;
  joint.InPortA.a       = aA;
  joint.InPortA.T       = TA;
  joint.InPortA.omega   = omegaA;
  joint.InPortA.epsilon = epsilonA;

  joint.InPortB.r       = rB;
  joint.InPortB.v       = vB;
  joint.InPortB.a       = aB;
  joint.InPortB.T       = TB;
  joint.InPortB.omega   = omegaB;
  joint.InPortB.epsilon = epsilonB;

  // get the information out of the constraint and pass it into dynamics

  rA = joint.OutPortA.P;
  FA = joint.OutPortA.F;
  MA = joint.OutPortA.M;

  rB = joint.OutPortB.P;
  FB = joint.OutPortB.F;
  MB = joint.OutPortB.M;

  when initial() then
    terminate("passed");
  end when;

  annotation(experiment
    ( StopTime = 10
    , NumberOfIntervals = 100
    , Tolerance = 1e-9
    , Algorithm = "Dassl"
    ));

  when time > 0 then
    assert
      ( false
      , "should have terminated after checking the solution to the initial problem"
      );
  end when;

end JointTestExplicit;
