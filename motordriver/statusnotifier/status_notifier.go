package statusnotifier

//getDriverStatusNotifier returns the specialized status notifier
//for now returns ucam's status notifier defined in ucamDriverStatusNotifier.go
func getDriverStatusNotifier() IDriverStatusNotifier {
	return &UCAMNotifier{}
}

//NotifyCurrentPosition notify the current position to client
func NotifyCurrentPosition(driveName string, currentPos float64) {
	notifier := getDriverStatusNotifier()
	notifier.CurrentPosition(driveName, currentPos)
}

//NotifyDestinationPosition notify destination position to client
func NotifyDestinationPosition(driveName string, destPos float32) {
	notifier := getDriverStatusNotifier()
	notifier.DestinationPosition(driveName, destPos)
}

//Alarm raise alarm to the ui
func Alarm(alarm string) {
	notifier := getDriverStatusNotifier()
	notifier.Alarm(alarm)
}

//DriverError send driver error to the connected clients
func DriverError(errorCode int) {
	notifier := getDriverStatusNotifier()
	notifier.DriverError(errorCode)
}

func DriverStatus(driveName, status string) {
	notifier := getDriverStatusNotifier()
	notifier.DriverStatus(driveName, status)
}

func NotifyIOStatus(ioStat IOStatus) {
	notifier := getDriverStatusNotifier()
	notifier.NotifyIOStatus(ioStat)
}

func SocketMessage(event string, msg string) {
	notifier := getDriverStatusNotifier()
	notifier.SocketMessage(event, msg)
}
