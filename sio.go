//go:build linux
// +build linux

/*

1) Original: Copyright (c) 2005-2008 Dustin Sallings <dustin@spy.net>.

2) Mods: Copyright (c) 2012 Schleibinger Geräte Teubert u. Greim GmbH
<info@schleibinger.com>. Blame: Jan Mercl

All rights reserved.  Use of this source code is governed by a MIT-style
license that can be found in the LICENSE file.

*/

// Package sio supports communication using a serial port. Currently works only
// on Linux. Cgo is not used.
package sio

import (
	"os"
	"syscall"
	"time"
	"unsafe"

	"github.com/pkg/term/termios"
)

// Addr represents a network end point address.
type Addr interface {
	Network() string // name of the network
	String() string  // string form of address
}

type addr struct {
	net string
	str string
}

// Implementation of Addr
func (a *addr) Network() string {
	return a.net
}

// Implementation of Addr
func (a *addr) String() string {
	return a.str
}

type Port struct {
	f *os.File
	a *addr
}

const (
	termios_POSIX_VDISABLE = 0xff
	termios_CCTS_OFLOW     = 0x00010000
	termios_CRTS_IFLOW     = 0x00020000
	termios_CDTR_IFLOW     = 0x00040000
	termios_CDSR_OFLOW     = 0x00080000
	termios_CMSPAR         = 0x40000000
)

type RS485Params struct {
	ENABLED               bool
	RTS_ON_SEND           bool
	RTS_AFTER_SEND        bool
	RX_DURING_TX          bool
	TERMINATE_BUS         bool
	ADDRB                 bool
	ADDR_RECV             bool
	ADDR_DEST             bool
	MODE_RS422            bool
	DELAY_RTS_BEFORE_SEND uint32
	DELAY_RTS_AFTER_SEND  uint32
}

type FlowControlSettings struct {
	RS485Params RS485Params
	EnableRTS   bool
	EnableCTS   bool
	EnableDTR   bool
	EnableDSR   bool
}

// Open returns a Port implementing net.Conn or an error if any. The Port
// behavior is like of the merged returns of net.DialTCP and
// net.ListenTCP.Accept, i.e. the net.Conn represents a bidirectional byte
// stream. The only supported mode ATM is 8N1. The serial line is put into raw
// mode (e.g. no HW nor XON/XOFF flow control).
//
// Ex.: sio.Open("/dev/ttyS0", syscall.B115200)
func Open(dev string, rate uint32) (p *Port, err error) {
	return openPort(dev, rate, nil, FlowControlSettings{})
}

// OpenWithTimeout works like Open but sets a timeout for Read() operations.
// Set the timeout to 0 for non-blocking operation.
func OpenWithTimeout(dev string, rate uint32, readTimeout time.Duration, fcs FlowControlSettings) (p *Port, err error) {
	return openPort(dev, rate, &readTimeout, fcs)
}

func openPort(dev string, rate uint32, readTimeout *time.Duration, fcs FlowControlSettings) (p *Port, err error) {
	var f *os.File
	defer func() {
		if err != nil && f != nil {
			f.Close()
		}
	}()
	f, err = os.OpenFile(dev, syscall.O_RDWR|syscall.O_NOCTTY|syscall.O_NDELAY, 0666)
	if err != nil {
		return nil, err
	}

	cflag := syscall.CS8 | syscall.CREAD | syscall.CLOCAL | rate

	if fcs.EnableRTS {
		cflag |= termios_CRTS_IFLOW
	}

	if fcs.EnableCTS {
		cflag |= termios_CCTS_OFLOW
	}

	if fcs.EnableDTR {
		cflag |= termios_CDTR_IFLOW
	}

	if fcs.EnableDSR {
		cflag |= termios_CDSR_OFLOW
	}

	fd := f.Fd()
	t := syscall.Termios{
		Iflag:  syscall.IGNPAR,
		Cflag:  cflag,
		Cc:     [32]uint8{syscall.VMIN: 1},
		Ispeed: rate,
		Ospeed: rate,
	}
	if readTimeout != nil {
		vmin, vtime := posixTimeoutValues(*readTimeout)
		t.Cc[syscall.VMIN] = vmin
		t.Cc[syscall.VTIME] = vtime
	}

	if _, _, errno := syscall.Syscall6(
		syscall.SYS_IOCTL,
		uintptr(fd),
		uintptr(syscall.TCSETS),
		uintptr(unsafe.Pointer(&t)),
		0,
		0,
		0,
	); errno != 0 {
		return nil, errno
	}

	if err = syscall.SetNonblock(int(fd), false); err != nil {
		return
	}

	p = &Port{f, &addr{dev, dev}}

	if fcs.RS485Params.ENABLED {
		err = p.setRS485Params(&fcs.RS485Params)
		if err != nil {
			return
		}
	}

	return p, nil
}

func (p *Port) Sync() error {
	fd := p.f.Fd()

	return termios.Tcdrain(fd)
}

// Implementation of net.Conn
func (p *Port) Read(b []byte) (n int, err error) {
	return p.f.Read(b)
}

// Implementation of net.Conn
func (p *Port) Write(b []byte) (n int, err error) {
	return p.f.Write(b)
}

// Implementation of net.Conn
func (p *Port) Close() error {
	return p.f.Close()
}

// Implementation of net.Conn
func (p *Port) LocalAddr() Addr {
	return p.a
}

// Implementation of net.Conn
func (p *Port) RemoteAddr() Addr {
	return &addr{} // Ignored
}

// Implementation of net.Conn
func (p *Port) SetDeadline(t time.Time) error {
	return nil // Ignored
}

// Implementation of net.Conn
func (p *Port) SetReadDeadline(t time.Time) error {
	return nil // Ignored
}

// Implementation of net.Conn
func (p *Port) SetWriteDeadline(t time.Time) error {
	return nil // Ignored
}

type kernelRS485Struct struct {
	Flags              uint32
	DelayRTSBeforeSend uint32
	DelayRTSAfterSend  uint32
	Padding            [5]uint32
}

func (p *Port) setRS485Params(params *RS485Params) (err error) {
	var ctl kernelRS485Struct
	fd := p.f.Fd()

	ctl.DelayRTSBeforeSend = params.DELAY_RTS_BEFORE_SEND
	ctl.DelayRTSAfterSend = params.DELAY_RTS_AFTER_SEND

	ctl.Flags = 0
	ctl.Flags |= (boolToUint32(params.ENABLED) << 0)
	ctl.Flags |= (boolToUint32(params.RTS_ON_SEND) << 1)
	ctl.Flags |= (boolToUint32(params.RTS_AFTER_SEND) << 2)
	ctl.Flags |= (boolToUint32(params.RX_DURING_TX) << 4)
	ctl.Flags |= (boolToUint32(params.TERMINATE_BUS) << 5)
	ctl.Flags |= (boolToUint32(params.ADDRB) << 6)
	ctl.Flags |= (boolToUint32(params.ADDR_RECV) << 7)
	ctl.Flags |= (boolToUint32(params.ADDR_DEST) << 8)
	ctl.Flags |= (boolToUint32(params.MODE_RS422) << 9)

	if _, _, errno := syscall.Syscall6(
		syscall.SYS_IOCTL,
		uintptr(fd),
		uintptr(syscall.TIOCGRS485),
		uintptr(unsafe.Pointer(&ctl)),
		0,
		0,
		0,
	); errno != 0 {
		return errno
	}

	return nil
}

func (p *Port) setCtrlSignal(sig int, on bool) (err error) {
	var state int
	fd := p.f.Fd()

	if _, _, errno := syscall.Syscall6(
		syscall.SYS_IOCTL,
		uintptr(fd),
		uintptr(syscall.TIOCMGET),
		uintptr(unsafe.Pointer(&state)),
		0,
		0,
		0,
	); errno != 0 {
		return errno
	}

	switch on {
	case true:
		state |= sig
	case false:
		state &^= sig
	}

	if _, _, errno := syscall.Syscall6(
		syscall.SYS_IOCTL,
		uintptr(fd),
		uintptr(syscall.TIOCMSET),
		uintptr(unsafe.Pointer(&state)),
		0,
		0,
		0,
	); errno != 0 {
		err = errno
	}
	return
}

func (p *Port) getCtrlSignal(sig int) (on bool, err error) {
	var state int

	if _, _, errno := syscall.Syscall6(
		syscall.SYS_IOCTL,
		uintptr(p.f.Fd()),
		uintptr(syscall.TIOCMGET),
		uintptr(unsafe.Pointer(&state)),
		0,
		0,
		0,
	); errno != 0 {
		return false, errno
	}

	on = (state & sig) != 0
	return
}

// GetDTR return the state of p's DTR or an error if any.  Depending on the
// setup this signal may have the opposite direction than expected.  In such
// case this function should not be used.
func (p *Port) GetDTR() (on bool, err error) {
	return p.getCtrlSignal(syscall.TIOCM_DTR)
}

// GetDSR return the state of p's DSR or an error if any.  Depending on the
// setup this signal may have the opposite direction than expected.  In such
// case this function should not be used.
func (p *Port) GetDSR() (on bool, err error) {
	return p.getCtrlSignal(syscall.TIOCM_DSR)
}

// GetCTS return the state of p's CTS or an error if any.  Depending on the
// setup this signal may have the opposite direction than expected.  In such
// case this function should not be used.
func (p *Port) GetCTS() (on bool, err error) {
	return p.getCtrlSignal(syscall.TIOCM_CTS)
}

// GetRTS return the state of p's RTS or an error if any.  Depending on the
// setup this signal may have the opposite direction than expected.  In such
// case this function should not be used.
func (p *Port) GetRTS() (on bool, err error) {
	return p.getCtrlSignal(syscall.TIOCM_RTS)
}

// SetDTR sets the state of p's DTR to `on`. A non nil error is returned on
// failure.  Depending on the setup this signal may have the opposite direction
// than expected.  In such case this function should not be used.
func (p *Port) SetDTR(on bool) error {
	return p.setCtrlSignal(syscall.TIOCM_DTR, on)
}

// SetDSR sets the state of p's DSR to `on`. A non nil error is returned on
// failure.  Depending on the setup this signal may have the opposite direction
// than expected.  In such case this function should not be used.
func (p *Port) SetDSR(on bool) error {
	return p.setCtrlSignal(syscall.TIOCM_DSR, on)
}

// SetCTS sets the state of p's CTS to `on`. A non nil error is returned on
// failure.  Depending on the setup this signal may have the opposite direction
// than expected.  In such case this function should not be used.
func (p *Port) SetCTS(on bool) error {
	return p.setCtrlSignal(syscall.TIOCM_CTS, on)
}

// SetRTS sets the state of p's RTS to `on`. A non nil error is returned on
// failure.  Depending on the setup this signal may have the opposite direction
// than expected.  In such case this function should not be used.
func (p *Port) SetRTS(on bool) error {
	return p.setCtrlSignal(syscall.TIOCM_RTS, on)
}

// Converts the timeout values for Linux / POSIX systems
func posixTimeoutValues(readTimeout time.Duration) (vmin uint8, vtime uint8) {
	const MAXUINT8 = 1<<8 - 1 // 255
	// set blocking / non-blocking read
	var minBytesToRead uint8 = 1
	var readTimeoutInDeci int64
	if readTimeout > 0 {
		// EOF on zero read
		minBytesToRead = 0
		// convert timeout to deciseconds as expected by VTIME
		readTimeoutInDeci = (readTimeout.Nanoseconds() / 1e6 / 100)
		// capping the timeout
		if readTimeoutInDeci < 1 {
			// min possible timeout 1 Deciseconds (0.1s)
			readTimeoutInDeci = 1
		} else if readTimeoutInDeci > MAXUINT8 {
			// max possible timeout is 255 deciseconds (25.5s)
			readTimeoutInDeci = MAXUINT8
		}
	}
	return minBytesToRead, uint8(readTimeoutInDeci)
}

func boolToUint32(v bool) uint32 {
	if v {
		return 1
	} else {
		return 0
	}
}
