package rb

import (
	"bufio"
	"bytes"
	"encoding/binary"
	"errors"
	"fmt"
	"io"
)

// PacketHeaderMagic is the first byte of every packet on the data port.
const PacketHeaderMagic = 0x24

// PacketTypeSystemState is the type byte (header[3]) of a systemSTAT packet.
const PacketTypeSystemState = 0x03

// SystemStateSize is the size in bytes of the systemSTAT packet the control box
// sends in response to "reqdata", header included. The layout has only ever grown
// by appending fields (464 bytes in the 2020 ROS 1 driver), so the decoder
// tolerates shorter and longer packets.
const SystemStateSize = 580

// Robot (motion) states reported in SystemState.RobotState.
const (
	RobotStateIdle          = 1
	RobotStateMoving        = 3
	RobotStateIdleExtCtrl   = 5  // idle but under conveyor or force control
	RobotStateMovePBBase    = 60 // 60+index while executing MovePB/ITPL/Pro
	RobotStateMoveXBBase    = 560
	InitStateActivated      = 6
	TaskStateIdle           = 1
	TaskStatePaused         = 2
	TaskStateRunning        = 3
	OperationModeReal       = 0
	OperationModeSimulation = 1
)

// Bits of SystemState.InformationChunk1.
const (
	InfoBit48VIn         = 1 << 0
	InfoBit48VOut        = 1 << 1
	InfoBit24VIn         = 1 << 2
	InfoBitEStop1        = 1 << 3
	InfoBitUserSwitch    = 1 << 4
	InfoBitEStop2        = 1 << 5
	InfoBitArmPower      = 1 << 6
	InfoBitDirectTeachBt = 1 << 7
)

// Bits of SystemState.JntInfo[i].
const (
	JointBitFET       = 1 << 0
	JointBitRun       = 1 << 1
	JointBitInit      = 1 << 2
	JointBitMode      = 1 << 3
	JointBitNoniusErr = 1 << 4
	JointBitLowBatt   = 1 << 5
	JointBitCalib     = 1 << 6
	JointBitMultiTurn = 1 << 7
	JointBitJAM       = 1 << 8
	JointBitCUR       = 1 << 9
	JointBitBIG       = 1 << 10
	JointBitINP       = 1 << 11
	JointBitFLT       = 1 << 12
	JointBitTMP       = 1 << 13
	JointBitPS1       = 1 << 14
	JointBitPS2       = 1 << 15
)

// jointFaultNames maps each error bit of jnt_info to a short name.
var jointFaultNames = []struct {
	bit  int32
	name string
}{
	{JointBitJAM, "JAM"},
	{JointBitCUR, "CUR"},
	{JointBitBIG, "BIG"},
	{JointBitINP, "INP"},
	{JointBitFLT, "FLT"},
	{JointBitTMP, "TMP"},
	{JointBitPS1, "PS1"},
	{JointBitPS2, "PS2"},
}

// SystemState is the systemSTAT packet served on port 5001. Field order, types
// and sizes mirror rbpodo's data_type.hpp exactly; the struct is little-endian
// and packed on the wire. Angles are degrees, positions millimetres.
type SystemState struct {
	Header               [4]byte
	Time                 float32
	JntRef               [6]float32 // commanded joint angle (deg); what the ROS 2 driver publishes as position
	JntAng               [6]float32 // encoder joint angle (deg); frozen in simulation mode
	JntCur               [6]float32 // joint current (A)
	TCPRef               [6]float32 // commanded TCP x,y,z (mm), rx,ry,rz (deg, Euler Z-Y'-X'')
	TCPPos               [6]float32 // measured TCP
	AnalogIn             [4]float32
	AnalogOut            [4]float32
	DigitalIn            [16]int32
	DigitalOut           [16]int32
	JntTemperature       [6]float32
	TaskPC               int32
	TaskRepeat           int32
	TaskRunID            int32
	TaskRunNum           int32
	TaskRunTime          int32
	TaskState            int32 // 1 idle, 3 running, 2 paused
	DefaultSpeed         float32
	RobotState           int32 // 1 idle, 3 moving, 5 idle under external control, 60+ MovePB
	InformationChunk1    int32
	Reserved1            [6]float32
	JntInfo              [6]int32
	CollisionDetectOnOff int32
	IsFreedriveMode      int32
	RealVsSimulationMode int32 // 0 real, 1 simulation
	InitStateInfo        int32 // 6 = activated
	InitError            int32
	TFBAnalogIn          [2]float32
	TFBDigitalIn         [2]int32
	TFBDigitalOut        [2]int32
	TFBVoltageOut        float32
	OpStatCollisionOccur int32
	OpStatSOSFlag        int32
	OpStatSelfCollision  int32
	OpStatSoftEstopOccur int32
	OpStatEMSFlag        int32
	InformationChunk2    int32
	InformationChunk3    int32
	InboxTrapFlag        [2]int32
	InboxCheckMode       [2]int32
	EFTFx                float32
	EFTFy                float32
	EFTFz                float32
	EFTMx                float32
	EFTMy                float32
	EFTMz                float32
	InformationChunk4    int32
	ExtendIO1AnalogIn    [4]float32
	ExtendIO1AnalogOut   [4]float32
	ExtendIO1DigitalInfo uint32
	AAJointRef           [6]float32
	SafetyBoardStatInfo  uint32
}

// ErrBadPacket is returned when the bytes do not start with a systemSTAT header.
var ErrBadPacket = errors.New("rb: bad state packet")

// ErrShortPacket is returned when a packet is truncated relative to its own size header.
var ErrShortPacket = errors.New("rb: short state packet")

// PacketLength returns the total length (header included) declared by a packet
// header, or an error if the four header bytes do not describe a systemSTAT packet.
func PacketLength(header []byte) (int, error) {
	if len(header) < 4 {
		return 0, ErrShortPacket
	}
	if header[0] != PacketHeaderMagic || header[3] != PacketTypeSystemState {
		return 0, fmt.Errorf("%w: header % x", ErrBadPacket, header[:4])
	}
	size := int(header[1]) | int(header[2])<<8
	return size + 4, nil
}

// DecodeSystemState decodes one packet (header included). Packets shorter than
// SystemStateSize decode their leading fields and leave the rest zero; longer
// packets are accepted and their tail ignored. The packet must be complete with
// respect to its own size header.
func DecodeSystemState(pkt []byte) (SystemState, error) {
	var st SystemState
	total, err := PacketLength(pkt)
	if err != nil {
		return st, err
	}
	if len(pkt) < total {
		return st, fmt.Errorf("%w: have %d bytes, header says %d", ErrShortPacket, len(pkt), total)
	}
	buf := make([]byte, SystemStateSize)
	copy(buf, pkt[:min(total, SystemStateSize)])
	if err := binary.Read(bytes.NewReader(buf), binary.LittleEndian, &st); err != nil {
		return st, err
	}
	return st, nil
}

// EncodeSystemState produces a wire packet for st, filling in the header. It is
// used by the fake control box and by tests.
func EncodeSystemState(st SystemState) []byte {
	size := SystemStateSize - 4
	st.Header = [4]byte{PacketHeaderMagic, byte(size & 0xff), byte(size >> 8), PacketTypeSystemState}
	var buf bytes.Buffer
	_ = binary.Write(&buf, binary.LittleEndian, &st)
	return buf.Bytes()
}

// ReadPacket reads one systemSTAT packet from r, skipping any leading bytes that
// are not a valid header (the box has no framing beyond the header, so stray
// bytes after a reconnect must be discarded rather than fail the connection).
func ReadPacket(r *bufio.Reader) ([]byte, error) {
	for {
		b, err := r.ReadByte()
		if err != nil {
			return nil, err
		}
		if b != PacketHeaderMagic {
			continue
		}
		rest, err := r.Peek(3)
		if err != nil {
			return nil, err
		}
		if rest[2] != PacketTypeSystemState {
			// Not a packet start; keep scanning from the next byte.
			continue
		}
		total := int(rest[0]) | int(rest[1])<<8 + 4
		pkt := make([]byte, total)
		pkt[0] = b
		if _, err := io.ReadFull(r, pkt[1:]); err != nil {
			return nil, err
		}
		return pkt, nil
	}
}

// Activated reports whether the arm has completed activation.
func (s *SystemState) Activated() bool { return s.InitStateInfo&0x3f == InitStateActivated }

// Simulation reports whether the controller is in simulation (pgmode simulation) mode.
func (s *SystemState) Simulation() bool { return s.RealVsSimulationMode == OperationModeSimulation }

// Moving reports whether a motion command is executing.
func (s *SystemState) Moving() bool {
	return s.RobotState == RobotStateMoving || s.RobotState >= RobotStateMovePBBase
}

// Idle reports whether no motion command is executing.
func (s *SystemState) Idle() bool { return !s.Moving() }

// EStop reports whether the e-stop is engaged. Rainbow documents bits 3 and 5
// only as "E-Stop state 1/2"; the inputs are normally-closed safety channels
// that read 1 while the circuit is intact (button released) and drop to 0 when
// the button is pressed, which is what a powered, activated arm with the
// button released reports. Either channel reading 0 counts as engaged.
func (s *SystemState) EStop() bool {
	ch1, ch2 := s.EStopChannels()
	return !ch1 || !ch2
}

// EStopChannels returns the raw state of the two e-stop inputs (true = circuit
// closed, i.e. not pressed).
func (s *SystemState) EStopChannels() (ch1, ch2 bool) {
	return s.InformationChunk1&InfoBitEStop1 != 0, s.InformationChunk1&InfoBitEStop2 != 0
}

// ArmPowered reports whether power is applied to the arm.
func (s *SystemState) ArmPowered() bool { return s.InformationChunk1&InfoBitArmPower != 0 }

// Freedrive reports whether direct teaching (freedrive) is on.
func (s *SystemState) Freedrive() bool { return s.IsFreedriveMode == 1 }

// CollisionStopped reports whether an external collision has been detected and not cleared.
func (s *SystemState) CollisionStopped() bool { return s.OpStatCollisionOccur != 0 }

// JointFaults returns a human-readable list of joint fault flags, one entry per
// faulted joint, or nil when no joint reports an error bit.
func (s *SystemState) JointFaults() []string {
	var out []string
	for i, info := range s.JntInfo {
		var names []string
		for _, f := range jointFaultNames {
			if info&f.bit != 0 {
				names = append(names, f.name)
			}
		}
		if len(names) > 0 {
			out = append(out, fmt.Sprintf("joint %d: %v", i, names))
		}
	}
	return out
}

// EMSDescription describes op_stat_ems_flag.
func (s *SystemState) EMSDescription() string {
	switch s.OpStatEMSFlag {
	case 0:
		return ""
	case 1:
		return "arm stretch"
	case 2:
		return "cartesian limit"
	case 3:
		return "joint limit"
	case 4:
		return "unsolvable"
	default:
		return fmt.Sprintf("code %d", s.OpStatEMSFlag)
	}
}

// SOSDescription describes op_stat_sos_flag.
func (s *SystemState) SOSDescription() string {
	names := []string{"", "encoder err (PVL)", "CPU err", "big err", "input err", "JAM err", "over current err",
		"position bound err", "mode err", "match err", "over current/low voltage err", "temperature err", "speed over err"}
	if int(s.OpStatSOSFlag) < len(names) && s.OpStatSOSFlag >= 0 {
		return names[s.OpStatSOSFlag]
	}
	return fmt.Sprintf("code %d", s.OpStatSOSFlag)
}

// Joints returns jnt_ref as float64 degrees.
func (s *SystemState) Joints() [6]float64 {
	var j [6]float64
	for i := range j {
		j[i] = float64(s.JntRef[i])
	}
	return j
}

// EncoderJoints returns jnt_ang as float64 degrees.
func (s *SystemState) EncoderJoints() [6]float64 {
	var j [6]float64
	for i := range j {
		j[i] = float64(s.JntAng[i])
	}
	return j
}
