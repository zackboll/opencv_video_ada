with AUnit.Run;
with AUnit.Reporter.Text;
with Ada.Command_Line;
with Video_Tests;

procedure Run_Tests is
   function Runner is new AUnit.Run.Test_Runner_With_Status (Video_Tests.Suite);
   Reporter : AUnit.Reporter.Text.Text_Reporter;
   use type AUnit.Status;
begin
   if Runner (Reporter) /= AUnit.Success then
      Ada.Command_Line.Set_Exit_Status (Ada.Command_Line.Failure);
   end if;
end Run_Tests;
