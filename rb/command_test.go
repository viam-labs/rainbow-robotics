package rb

import (
	"regexp"
	"testing"
)

func TestFormatters(t *testing.T) {
	j := [6]float64{0, 45, 90, 0, 90, 0}
	cases := []struct{ got, want string }{
		{Activate(), "mc jall init"},
		{Shutdown(), "AvadaKedavra()"},
		{ControlBoxInfo(), "get_control_box_info()"},
		{PGMode(true), "pgmode real"},
		{PGMode(false), "pgmode simulation"},
		{SetSpeedBar(0.5), "set_speed_bar(0.5)"},
		{MoveJ(j, 60, 80), "move_j(jnt[0, 45, 90, 0, 90, 0],60,80)"},
		{MoveJB2Clear(), "move_jb2_clear()"},
		{MoveJB2Add(j, 60, 80, 0.5), "move_jb2_add(jnt[0, 45, 90, 0, 90, 0],60,80,0,0.5)"},
		{MoveJB2Run(), "move_jb2_run()"},
		{MoveServoJ(j, 0.01, 0.03, 0.5, 0.5), "move_servo_j(jnt[0, 45, 90, 0, 90, 0],0.01,0.03,0.5,0.5)"},
		{MoveSpeedJ([6]float64{}, 0.01, 0.03, 0.5, 0.5), "move_speed_j(jnt[0, 0, 0, 0, 0, 0],0.01,0.03,0.5,0.5)"},
		{TaskStop(), "task stop"},
		{TaskPause(), "task pause"},
		{TaskResume(false), "task resume_a"},
		{TaskResume(true), "task resume_b"},
		{Freedrive(true), "freedrive_teach_on()"},
		{Freedrive(false), "freedrive_teach_off()"},
		{SetCollisionOnOff(true), "set_collision_onoff(1)"},
		{SetCollisionOnOff(false), "set_collision_onoff(0)"},
		{SetCollisionThreshold(0.7), "set_collision_th(0.7)"},
		{SetCollisionMode(1), "set_collision_mode(1)"},
		{SetCollisionAfter(0), "set_collision_after(0)"},
		{SetTCPInfo([6]float64{1, 2, 3, 4, 5, 6}), "set_tcp_info(1,2,3,4,5,6)"},
		{SetPayloadInfo(2.5, 0, 0, 100), "set_payload_info(2.5,0,0,100)"},
		{ToolOut(24, 1, 0), "tool_out 24,1,0"},
		{SetBoxDOut(3, 1), "set_box_dout(3,1)"},
		{SetBoxDOut(3, -1), "set_box_dout(3,-1)"},
		{SetBoxAOut(0, 2.5), "set_box_aout(0,2.5)"},
		{PrintTagged("SD_ROBOT_STATE", "#ab12c_"), `print(SD_ROBOT_STATE,"#ab12c_[", "]")`},
		{Print(`"enable_waiting_ack"`, "", ""), `print("enable_waiting_ack","", "")`},
		{CalcFKTCP(j), "calc_fk_tcp(0,45,90,0,90,0)"},
		// No exponent notation, ever.
		{Num(0.000001), "0.000001"},
		{Num(1e21), "1000000000000000000000"},
		{Num(-33.333333333333336), "-33.333333333333336"},
	}
	for _, c := range cases {
		if c.got != c.want {
			t.Errorf("got  %q\nwant %q", c.got, c.want)
		}
	}
}

func TestNewTag(t *testing.T) {
	re := regexp.MustCompile(`^#[0-9a-z]{5}_$`)
	seen := map[string]bool{}
	for range 50 {
		tag := NewTag()
		if !re.MatchString(tag) {
			t.Fatalf("bad tag %q", tag)
		}
		seen[tag] = true
	}
	if len(seen) < 40 {
		t.Errorf("tags not random enough: %d unique of 50", len(seen))
	}
}

func TestParseFloats(t *testing.T) {
	got, err := ParseFloats("1.0, 2, -3.5, 4, 5, 6")
	if err != nil {
		t.Fatal(err)
	}
	if len(got) != 6 || got[2] != -3.5 {
		t.Errorf("got %v", got)
	}
	got, err = ParseFloats("jnt[1, 2, 3, 4, 5, 6]")
	if err != nil || len(got) != 6 {
		t.Errorf("jnt form: %v %v", got, err)
	}
	if _, err := ParseFloats("1, x"); err == nil {
		t.Error("expected error")
	}
}
