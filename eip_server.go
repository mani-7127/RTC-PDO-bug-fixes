package main

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"io"
	"log"
	"net"
	"sync/atomic"
)

const (
	cmdListServices    uint16 = 0x0004
	cmdRegisterSession uint16 = 0x0065
	cmdUnregister      uint16 = 0x0066
	cmdSendRRData      uint16 = 0x006F
)

type encapsulationHeader struct {
	Command       uint16
	Length        uint16
	SessionHandle uint32
	Status        uint32
	SenderContext [8]byte
	Options       uint32
}

var nextSession uint32 = 1000

func main() {
	listener, err := net.Listen("tcp", ":44818")
	if err != nil {
		log.Fatalf("listen failed: %v", err)
	}
	defer listener.Close()

	log.Println("EtherNet/IP 2-way POC server listening on TCP 44818")

	for {
		conn, err := listener.Accept()
		if err != nil {
			log.Printf("accept error: %v", err)
			continue
		}
		go handleConnection(conn)
	}
}

func handleConnection(conn net.Conn) {
	defer conn.Close()
	log.Printf("client connected: %s", conn.RemoteAddr())

	for {
		var header encapsulationHeader
		if err := binary.Read(conn, binary.LittleEndian, &header); err != nil {
			if err != io.EOF {
				log.Printf("read header error: %v", err)
			}
			log.Printf("client disconnected: %s", conn.RemoteAddr())
			return
		}

		payload := make([]byte, header.Length)
		if _, err := io.ReadFull(conn, payload); err != nil {
			log.Printf("read payload error: %v", err)
			return
		}

		switch header.Command {
		case cmdRegisterSession:
			handleRegisterSession(conn, header, payload)

		case cmdSendRRData:
			handleSendRRData(conn, header, payload)

		case cmdUnregister:
			log.Printf("unregistered session %d from %s", header.SessionHandle, conn.RemoteAddr())
			return

		case cmdListServices:
			responsePayload := minimalListServicesPayload()
			responseHeader := encapsulationHeader{
				Command:       cmdListServices,
				Length:        uint16(len(responsePayload)),
				SessionHandle: header.SessionHandle,
				Status:        0,
				SenderContext: header.SenderContext,
				Options:       0,
			}
			if err := writePacket(conn, responseHeader, responsePayload); err != nil {
				log.Printf("write ListServices response error: %v", err)
				return
			}

		default:
			log.Printf("unsupported command 0x%04X from %s", header.Command, conn.RemoteAddr())
			writeError(conn, header, 0x0008)
		}
	}
}

func handleRegisterSession(conn net.Conn, header encapsulationHeader, payload []byte) {
	if len(payload) < 4 {
		writeError(conn, header, 0x0001)
		return
	}

	protocolVersion := binary.LittleEndian.Uint16(payload[0:2])
	if protocolVersion != 1 {
		writeError(conn, header, 0x0069)
		return
	}

	session := atomic.AddUint32(&nextSession, 1)

	responsePayload := make([]byte, 4)
	binary.LittleEndian.PutUint16(responsePayload[0:2], 1)
	binary.LittleEndian.PutUint16(responsePayload[2:4], 0)

	responseHeader := encapsulationHeader{
		Command:       cmdRegisterSession,
		Length:        uint16(len(responsePayload)),
		SessionHandle: session,
		Status:        0,
		SenderContext: header.SenderContext,
		Options:       0,
	}

	if err := writePacket(conn, responseHeader, responsePayload); err != nil {
		log.Printf("write RegisterSession response error: %v", err)
		return
	}

	log.Printf("registered session %d for %s", session, conn.RemoteAddr())
}

func handleSendRRData(conn net.Conn, header encapsulationHeader, payload []byte) {
	receivedMessage := string(payload)

	log.Printf("Received SendRRData from session %d", header.SessionHandle)
	log.Printf("Payload length: %d bytes", len(payload))
	log.Printf("Raw bytes: % X", payload)
	log.Printf("As string: %s", receivedMessage)

	replyMessage := fmt.Sprintf("I received your message: %s", receivedMessage)
	replyPayload := []byte(replyMessage)

	responseHeader := encapsulationHeader{
		Command:       cmdSendRRData,
		Length:        uint16(len(replyPayload)),
		SessionHandle: header.SessionHandle,
		Status:        0,
		SenderContext: header.SenderContext,
		Options:       0,
	}

	if err := writePacket(conn, responseHeader, replyPayload); err != nil {
		log.Printf("write SendRRData response error: %v", err)
		return
	}

	log.Printf("Reply sent: %s", replyMessage)
}

func writeError(conn net.Conn, requestHeader encapsulationHeader, status uint32) {
	responseHeader := encapsulationHeader{
		Command:       requestHeader.Command,
		Length:        0,
		SessionHandle: requestHeader.SessionHandle,
		Status:        status,
		SenderContext: requestHeader.SenderContext,
		Options:       0,
	}
	_ = writePacket(conn, responseHeader, nil)
}

func writePacket(conn net.Conn, header encapsulationHeader, payload []byte) error {
	buffer := new(bytes.Buffer)

	if err := binary.Write(buffer, binary.LittleEndian, header); err != nil {
		return err
	}

	if len(payload) > 0 {
		if _, err := buffer.Write(payload); err != nil {
			return err
		}
	}

	_, err := conn.Write(buffer.Bytes())
	return err
}

func minimalListServicesPayload() []byte {
	buffer := new(bytes.Buffer)

	_ = binary.Write(buffer, binary.LittleEndian, uint16(1))
	_ = binary.Write(buffer, binary.LittleEndian, uint16(0x0100))
	_ = binary.Write(buffer, binary.LittleEndian, uint16(20))
	_ = binary.Write(buffer, binary.LittleEndian, uint16(1))
	_ = binary.Write(buffer, binary.LittleEndian, uint16(0))

	name := make([]byte, 16)
	copy(name, []byte("EtherNet/IP POC"))
	_, _ = buffer.Write(name)

	return buffer.Bytes()
}