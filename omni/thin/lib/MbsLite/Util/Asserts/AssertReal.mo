within MbsLite.Util.Asserts;

function AssertReal

  input Real expected;
  input Real actual;
  input String name;
  input Boolean silent = false;

algorithm
 
  Assert
    ( CompareReal(expected, actual)
    , name + " should be: " + String(expected) + ", but was: " + String(actual)
    , silent = silent
    );

end AssertReal;
