within MbsLite.Examples.Omni;

model PlaneContact
 "A -- plane, B -- roller"
  extends Constraint;

  parameter String name                        = "NOT INITIALIZED";
  parameter Params params;
  parameter FrictionParams frictionParams;

  parameter Boolean isInContactInitially       = false;

  Real[3] contactPointCoords                        (each stateSelect = StateSelect.never);
  Real[3] contactPointVelocity                      (each stateSelect = StateSelect.never);
  Real    contactPointVelocityNorm                  (stateSelect = StateSelect.never);
  Real    cosBetweenWheelVerticalAndGlobalVertical  (stateSelect = StateSelect.never);

  Boolean isInContactNormal                         (stateSelect = StateSelect.never, start = isInContactInitially);
  Boolean isInContactFriction                       (stateSelect = StateSelect.never, start = isInContactInitially);
  Boolean isInContactKinematics                     (stateSelect = StateSelect.never, start = isInContactInitially);

  Real    normalVelocity                            (stateSelect = StateSelect.never);
  Real    DnormalVelocity                           (stateSelect = StateSelect.never);
  Real    normalReaction                            (stateSelect = StateSelect.never);

  Real[3] friction                                  (each stateSelect = StateSelect.never);
  Real    frictionNorm                              (stateSelect      = StateSelect.never);

  constant Real[3] rollerAxisLocal = forward;
  Real[3] rollerAxisGlobal                          (each stateSelect = StateSelect.never);
  Real[3] userwardHorizontalGlobal                  (each stateSelect = StateSelect.never);
  Real[3] towardsWheelCenterGlobal                  (each stateSelect = StateSelect.never);

equation

  cosBetweenWheelVerticalAndGlobalVertical
    = towardsWheelCenterGlobal * vertical;

  isInContactNormal
    = cosBetweenWheelVerticalAndGlobalVertical > cos
        ( params.rollerHalfAngle
        )
      and InPortB.r[2] < params.wheelRadius;
  
  isInContactFriction
    = cosBetweenWheelVerticalAndGlobalVertical > cos
        ( params.rollerHalfAngle
        - frictionParams.frictionGapAtEndOfRoller // trying to battle multiple impacts
        )
      and InPortB.r[2] < params.wheelRadius;

  isInContactKinematics
    = cosBetweenWheelVerticalAndGlobalVertical > cos
        ( params.rollerHalfAngle
        + frictionParams.frictionGapAtEndOfRoller // trying to battle sudden simulation interrupts
        )
      and InPortB.r[2] < params.wheelRadius;

  // CONTACT POINT COORDS AND VELOCITY
  rollerAxisGlobal = InPortB.T * rollerAxisLocal;
  userwardHorizontalGlobal = cross(rollerAxisGlobal, vertical);
  towardsWheelCenterGlobal = normalize(cross(userwardHorizontalGlobal, rollerAxisGlobal));

  contactPointCoords   = 
    if /*noEvent*/(isInContactNormal) then
      // start from roller center, go to wheel center and then outright downward.
      InPortB.r                                          
        + params.wheelHubRadius * towardsWheelCenterGlobal
        - params.wheelRadius * vertical
    else zeros(3);

  contactPointVelocity =
    if /*noEvent*/(isInContactNormal) then
      Euler
        ( InPortB.r
        , contactPointCoords
        , InPortB.v
        , InPortB.omega
        )
    else zeros(3);

  // SIGNORINI'S LAW FOR MOVEMENT ALONG THE NORMAL
  normalVelocity      = contactPointVelocity[2];
  der(normalVelocity) = DnormalVelocity;
  contactPointVelocityNorm = length(contactPointVelocity);

  if /*noEvent*/(isInContactNormal) then
    DnormalVelocity = 0;
  else
    normalReaction = 0;
  end if;

/*
  when isInContactNormal then
    reinit(normalVelocity, 0);
  end when;
*/

  // FRICTION
  if /*noEvent*/(isInContactFriction) then

    if FrictionType.none == frictionParams.frictionType then
      friction = zeros(3);

    elseif FrictionType.viscous == frictionParams.frictionType then
      friction = -frictionParams.viscousFrictionCoeff * contactPointVelocity
        ;

    elseif FrictionType.dry == frictionParams.frictionType then
      friction = -frictionParams.dryFrictionCoeff * contactPointVelocity
        * ( if /*noEvent*/(contactPointVelocityNorm <= frictionParams.viscousFrictionVelocityBound)
          then 1 / frictionParams.viscousFrictionVelocityBound
          else 1 / contactPointVelocityNorm
          )
        * normalReaction
        ;
    else
      assert(false, "Unsupported frictionType: " + String(frictionParams.frictionType));
      friction = zeros(3);
    end if;

    /*
    Assert
      ( CompareReal(0, contactPointVelocity[2], absTol = 1e-5)
      , name + " contact point has vertical speed!"
      );
    */
  else
    friction = zeros(3);
  end if;

  OutPortB.F = friction + normalReaction * vertical;

  OutPortA.P = { contactPointCoords[1], contactPointCoords[2], contactPointCoords[3] };

  OutPortB.P = contactPointCoords;
  OutPortB.M = zeros(3);

  frictionNorm = length(friction);

  // assert(/*noEvent*/(isInContact and CompareReal(0, normalVelocity) or not isInContact), "contact " + name + " is active, but there is normal velocity: " + String(normalVelocity));
end PlaneContact;

