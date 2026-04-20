package main

import (
	"bytes"
	"encoding/binary"
	"io"
	"log"
	"net"
	"sync/atomic"
)

const (
	cmdListServices    uint16 = 0x0004
	cmdRegisterSession uint16 = 0x0065
	cmdUnregister      uint16 = 0x0066
)

type encapHeader struct {
	Command       uint16
	Length        uint16
	SessionHandle uint32
	Status        uint32
	SenderContext [8]byte
	Options       uint32
}

var nextSession uint32 = 1001

func main() {
	ln, err := net.Listen("tcp", ":44818")
	if err != nil {
		log.Fatalf("listen failed: %v", err)
	}
	defer ln.Close()

	log.Println("EtherNet/IP POC server listening on TCP 44818")

	for {
		conn, err := ln.Accept()
		if err != nil {
			log.Printf("accept error: %v", err)
			continue
		}
		go handleConn(conn)
	}
}

func handleConn(conn net.Conn) {
	defer conn.Close()
	log.Printf("client connected: %s", conn.RemoteAddr())

	for {
		var hdr encapHeader
		if err := binary.Read(conn, binary.LittleEndian, &hdr); err != nil {
			if err != io.EOF {
				log.Printf("read header error: %v", err)
			}
			return
		}

		payload := make([]byte, hdr.Length)
		if _, err := io.ReadFull(conn, payload); err != nil {
			log.Printf("read payload error: %v", err)
			return
		}

		switch hdr.Command {
		case cmdRegisterSession:
			if len(payload) < 4 {
				writeError(conn, hdr, 0x0001)
				continue
			}

			protoVer := binary.LittleEndian.Uint16(payload[0:2])
			if protoVer != 1 {
				writeError(conn, hdr, 0x0069)
				continue
			}

			session := atomic.AddUint32(&nextSession, 1)

			respPayload := make([]byte, 4)
			binary.LittleEndian.PutUint16(respPayload[0:2], 1) // protocol version
			binary.LittleEndian.PutUint16(respPayload[2:4], 0) // options flags

			resp := encapHeader{
				Command:       cmdRegisterSession,
				Length:        uint16(len(respPayload)),
				SessionHandle: session,
				Status:        0,
				SenderContext: hdr.SenderContext,
				Options:       0,
			}
			if err := writePacket(conn, resp, respPayload); err != nil {
				log.Printf("write register response error: %v", err)
				return
			}
			log.Printf("registered session %d for %s", session, conn.RemoteAddr())

		case cmdUnregister:
			log.Printf("unregistered session %d from %s", hdr.SessionHandle, conn.RemoteAddr())
			return

		case cmdListServices:
			// Minimal placeholder response for POC
			respPayload := minimalListServicesPayload()

			resp := encapHeader{
				Command:       cmdListServices,
				Length:        uint16(len(respPayload)),
				SessionHandle: hdr.SessionHandle,
				Status:        0,
				SenderContext: hdr.SenderContext,
				Options:       0,
			}
			if err := writePacket(conn, resp, respPayload); err != nil {
				log.Printf("write list services response error: %v", err)
				return
			}

		default:
			log.Printf("unsupported command 0x%04X from %s", hdr.Command, conn.RemoteAddr())
			writeError(conn, hdr, 0x0008)
		}
	}
}

func writeError(conn net.Conn, req encapHeader, status uint32) {
	resp := encapHeader{
		Command:       req.Command,
		Length:        0,
		SessionHandle: req.SessionHandle,
		Status:        status,
		SenderContext: req.SenderContext,
		Options:       0,
	}
	_ = writePacket(conn, resp, nil)
}

func writePacket(conn net.Conn, hdr encapHeader, payload []byte) error {
	buf := new(bytes.Buffer)
	if err := binary.Write(buf, binary.LittleEndian, hdr); err != nil {
		return err
	}
	if len(payload) > 0 {
		if _, err := buf.Write(payload); err != nil {
			return err
		}
	}
	_, err := conn.Write(buf.Bytes())
	return err
}

func minimalListServicesPayload() []byte {
	// Very small POC payload: item count 1, one item entry
	// Not intended as a production-compliant stack.
	buf := new(bytes.Buffer)

	_ = binary.Write(buf, binary.LittleEndian, uint16(1))      // item count
	_ = binary.Write(buf, binary.LittleEndian, uint16(0x0100)) // type id: list services
	_ = binary.Write(buf, binary.LittleEndian, uint16(20))     // item length

	_ = binary.Write(buf, binary.LittleEndian, uint16(1)) // version
	_ = binary.Write(buf, binary.LittleEndian, uint16(0)) // capability flags

	name := make([]byte, 16)
	copy(name, []byte("EtherNet/IP POC"))
	_, _ = buf.Write(name)

	return buf.Bytes()
}