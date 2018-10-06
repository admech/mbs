within MbsLite.Util.Assert;

function AssertReals

  input Real[:] expected;
  input Real[:] actual;
  input String name;

protected
  Boolean passed = true;
  String message = name
    + " should be: " + StringA(expected)
    + ", but was: " + StringA(actual);

algorithm
 
  AssertReal
    ( size(expected, 1), size(actual, 1), "sizes of expected and actual"
    , silent = true
    );
  for i in 1 : size(actual, 1) loop
    passed := passed and CompareReal(expected[i], actual[i]);
    Assert(passed, message, silent = true);
  end for;
  if passed then
    print("Assertion passed: " + message);
  end if;

end AssertReals;
