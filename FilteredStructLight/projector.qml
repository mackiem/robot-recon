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
	//property int start_pos : Math.floor(Math.random() * background.width);
	//property int end_pos : ((Math.random() - 0.5) > 0) ? 0 : background.width;
	property int left_pt : parent.width / 3;
	property int right_pt : parent.width * 1.8 / 3;
	property int start_pos : left_pt;
	property int end_pos : right_pt;
	property bool left_to_right : true;
		anchors.top : background.top
		anchors.bottom : background.bottom
		//anchors.horizontalCenter : background.horizontalCenter
		
		width : 10
		color : "white"

		property int offset : 100

		/*
		SequentialAnimation on x {
			id: seq_animation
			running : true
			//loops : Animation.Infinite 
			PauseAnimation {
				duration : 1000
			}
			NumberAnimation {
				 target: moving_stripe; 
				 property: "x"; 
				 from : moving_stripe.start_pos
				 to: moving_stripe.start_pos + moving_stripe.offset
				 duration: 1
			}
			PauseAnimation { duration : 1000
			}

			onStopped : {
				if (!seq_animation.running) {
					if (((moving_stripe.start_pos) > moving_stripe.right_pt) || (moving_stripe.start_pos < moving_stripe.left_pt)) {
						moving_stripe.offset = -moving_stripe.offset
						console.log(moving_stripe.offset);
					}
					moving_stripe.start_pos += moving_stripe.offset
					seq_animation.start();
				}
			}
		}
		*/

		
	
		NumberAnimation on x {
			id : stripe_animation
			from : moving_stripe.start_pos
			to : moving_stripe.end_pos
			duration : Math.round(Math.abs(moving_stripe.start_pos - moving_stripe.end_pos) / background.speed);
			//duration : 10;
			//loops : Animation.Infinite
			onStopped : {
				//console.log("Value of " + Math.abs(moving_stripe.start_pos - moving_stripe.end_pos) / background.speed + "\n");
				//console.log("duration : " + stripe_animation.duration + "\n");
				if (!stripe_animation.running) {
	//moving_stripe.start_pos = Math.floor(Math.random() * background.width);
	//moving_stripe.end_pos = ((Math.random() - 0.5) > 0) ? 0 : background.width;
	if (moving_stripe.left_to_right) {
	moving_stripe.start_pos = moving_stripe.left_pt;
	moving_stripe.end_pos = moving_stripe.right_pt;
	} else {
	moving_stripe.start_pos = moving_stripe.right_pt;
	moving_stripe.end_pos = moving_stripe.left_pt;
	}
	moving_stripe.left_to_right = !moving_stripe.left_to_right;
	//moving_stripe.start_pos = background.width/2;
	//moving_stripe.end_pos = background.width/2;
	stripe_animation.duration = Math.round(Math.abs(moving_stripe.start_pos - moving_stripe.end_pos) / background.speed);
	stripe_animation.start();
				}
			}
		}
		
	
		
	}

}