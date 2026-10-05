package rb

import (
	"bufio"
	"bytes"
	"encoding/binary"
	"errors"
	"testing"
)

func TestSystemStateSize(t *testing.T) {
	if got := binary.Size(SystemState{}); got != SystemStateSize {
		t.Fatalf("binary.Size(SystemState{}) = %d, want %d", got, SystemStateSize)
	}
}

// buildPacket hand-assembles a packet from the documented offsets so the
// decoder is checked against the wire layout, not against its own encoder.
func buildPacket(total int) []byte {
	pkt := make([]byte, total)
	pkt[0] = 0x24
	size := total - 4
	pkt[1] = byte(size & 0xff)
	pkt[2] = byte(size >> 8)
	pkt[3] = 0x03
	putF := func(off int, v float32) { binary.LittleEndian.PutUint32(pkt[off:], mathFloat32bits(v)) }
	putI := func(off int, v int32) { binary.LittleEndian.PutUint32(pkt[off:], uint32(v)) }
	putF(4, 12.5)      // time
	for i := range 6 { // jnt_ref at 8
		putF(8+4*i, float32(10*(i+1)))
	}
	for i := range 6 { // jnt_ang at 32
		putF(32+4*i, float32(10*(i+1))+0.5)
	}
	for i := range 6 { // tcp_ref at 80
		putF(80+4*i, float32(100*(i+1)))
	}
	putI(312, 4)          // task_pc
	putI(332, 1)          // task_state
	putF(336, 0.75)       // default_speed
	putI(340, 3)          // robot_state
	putI(344, 0b01000111) // information_chunk_1: 48V in/out, 24V in, arm power
	putI(372+4*2, 1<<8)   // jnt_info[2] JAM
	putI(396, 1)          // collision_detect_onoff
	putI(400, 0)          // is_freedrive_mode
	putI(404, 1)          // real_vs_simulation_mode
	putI(408, 6)          // init_state_info
	putI(444, 1)          // op_stat_collision_occur
	if total >= 512 {
		putF(488, 1.5) // eft_fx
		putF(508, 6.5) // eft_mz
	}
	if total >= 580 {
		putF(552, 42) // aa_joint_ref[0]
		binary.LittleEndian.PutUint32(pkt[576:], 0xdeadbeef)
	}
	return pkt
}

func TestDecodeSystemStateGolden(t *testing.T) {
	st, err := DecodeSystemState(buildPacket(SystemStateSize))
	if err != nil {
		t.Fatal(err)
	}
	if st.Time != 12.5 {
		t.Errorf("Time = %v", st.Time)
	}
	if st.JntRef != [6]float32{10, 20, 30, 40, 50, 60} {
		t.Errorf("JntRef = %v", st.JntRef)
	}
	if st.JntAng[5] != 60.5 {
		t.Errorf("JntAng[5] = %v", st.JntAng[5])
	}
	if st.TCPRef[2] != 300 {
		t.Errorf("TCPRef = %v", st.TCPRef)
	}
	if st.TaskPC != 4 || st.TaskState != 1 || st.DefaultSpeed != 0.75 || st.RobotState != 3 {
		t.Errorf("task/robot fields wrong: %+v", st)
	}
	if !st.ArmPowered() || st.EStop() {
		t.Errorf("chunk1 bits wrong: %b", st.InformationChunk1)
	}
	if faults := st.JointFaults(); len(faults) != 1 || faults[0] != "joint 2: [JAM]" {
		t.Errorf("JointFaults = %v", faults)
	}
	if st.CollisionDetectOnOff != 1 || !st.Simulation() || !st.Activated() || !st.CollisionStopped() {
		t.Errorf("state flags wrong: %+v", st)
	}
	if st.EFTFx != 1.5 || st.EFTMz != 6.5 {
		t.Errorf("eft wrong: %v %v", st.EFTFx, st.EFTMz)
	}
	if st.AAJointRef[0] != 42 || st.SafetyBoardStatInfo != 0xdeadbeef {
		t.Errorf("tail fields wrong: %v %x", st.AAJointRef, st.SafetyBoardStatInfo)
	}
	if !st.Moving() {
		t.Error("expected moving")
	}
}

func TestDecodeSystemStateShortAndLong(t *testing.T) {
	// A 464-byte packet (2020 ROS 1 layout) decodes the leading fields, zeros the rest.
	st, err := DecodeSystemState(buildPacket(464))
	if err != nil {
		t.Fatal(err)
	}
	if st.RobotState != 3 || st.InitStateInfo != 6 {
		t.Errorf("leading fields wrong: %+v", st)
	}
	if st.EFTFx != 0 || st.AAJointRef[0] != 0 {
		t.Errorf("tail should be zero: %v %v", st.EFTFx, st.AAJointRef)
	}

	// A longer packet is accepted.
	long := buildPacket(640)
	st, err = DecodeSystemState(long)
	if err != nil {
		t.Fatal(err)
	}
	if st.AAJointRef[0] != 42 || st.RobotState != 3 {
		t.Errorf("long packet decode wrong: %+v", st)
	}

	// Truncated relative to the header fails.
	if _, err := DecodeSystemState(buildPacket(580)[:500]); !errors.Is(err, ErrShortPacket) {
		t.Errorf("expected ErrShortPacket, got %v", err)
	}

	// Wrong type byte fails.
	bad := buildPacket(580)
	bad[3] = 0x02
	if _, err := DecodeSystemState(bad); !errors.Is(err, ErrBadPacket) {
		t.Errorf("expected ErrBadPacket, got %v", err)
	}
}

func TestEncodeRoundTrip(t *testing.T) {
	var st SystemState
	st.JntRef = [6]float32{1, 2, 3, 4, 5, 6}
	st.RobotState = 1
	st.SafetyBoardStatInfo = 7
	pkt := EncodeSystemState(st)
	if len(pkt) != SystemStateSize {
		t.Fatalf("len = %d", len(pkt))
	}
	got, err := DecodeSystemState(pkt)
	if err != nil {
		t.Fatal(err)
	}
	got.Header = [4]byte{}
	if got != st {
		t.Errorf("round trip mismatch:\n got %+v\nwant %+v", got, st)
	}
}

func TestReadPacketFraming(t *testing.T) {
	p1 := buildPacket(580)
	p2 := buildPacket(464)
	var stream bytes.Buffer
	stream.Write([]byte("garbage$x")) // stray bytes, including a false magic
	stream.Write(p1)
	stream.Write([]byte{0x24, 0x00}) // a truncated false start
	stream.Write(p2)
	r := bufio.NewReaderSize(&stream, 16) // small buffer forces multi-read assembly

	got, err := ReadPacket(r)
	if err != nil {
		t.Fatal(err)
	}
	if !bytes.Equal(got, p1) {
		t.Errorf("first packet mismatch (len %d)", len(got))
	}
	got, err = ReadPacket(r)
	if err != nil {
		t.Fatal(err)
	}
	if !bytes.Equal(got, p2) {
		t.Errorf("second packet mismatch (len %d)", len(got))
	}
	if _, err := ReadPacket(r); err == nil {
		t.Error("expected EOF")
	}
}
