import QtQuick 2.0

Rectangle {
	id : background
	width : 100
	height : 100
	color : "black"
	anchors.fill : parent
	property var speed : 0.1

	Rectangle {
		id : moving_stripe
	property int start_pos : Math.floor(Math.random() * background.width);
	property int end_pos : ((Math.random() - 0.5) > 0) ? 0 : background.width;
		anchors.top : background.top
		anchors.bottom : background.bottom
		width : 10
		color : "white"

		NumberAnimation on x {
			id : stripe_animation
			from : moving_stripe.start_pos
			to : moving_stripe.end_pos
			duration : Math.round(Math.abs(moving_stripe.start_pos - moving_stripe.end_pos) / background.speed);
			//duration : 10;
			//loops : Animation.Infinite
			onStopped : {
				console.log("Value of " + Math.abs(moving_stripe.start_pos - moving_stripe.end_pos) / background.speed + "\n");
				console.log("duration : " + stripe_animation.duration + "\n");
				if (!stripe_animation.running) {
	moving_stripe.start_pos = Math.floor(Math.random() * background.width);
	moving_stripe.end_pos = ((Math.random() - 0.5) > 0) ? 0 : background.width;
	stripe_animation.duration = Math.round(Math.abs(moving_stripe.start_pos - moving_stripe.end_pos) / background.speed);
	stripe_animation.start();
				}
			}
		}
	}

}