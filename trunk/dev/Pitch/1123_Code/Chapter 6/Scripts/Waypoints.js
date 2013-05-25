//Name of the icon image
public var iconName : String = "wayIcon.psd";
//Radius of each way point - use for checking the collision detection with the enemy
public var radius : float = 1.0;
//Toggle this to make the enemy move by order from the first index to last index (Looping)
public var orderDirection : boolean = false;

//Get all the transfrom of the waypoint - including the the parent
private var waypoints : Transform[];
//Current waypoint index
private var int_wayIndex : int;
//Next waypoint index
private var int_nextIndex : int;
//Length of all waypoints
private var int_wayLength : int;
//Movement direction of the enemy to next waypoint
private var v3_direction : Vector3;
//Checking if the enemy hit the waypoint
private var b_isHitRadius : boolean;

//Set up all parameters before Initailize
public function Awake() : void {
	//Get all Transforms of the gameObject include the children and the transform of this gameObject
	waypoints = gameObject.GetComponentsInChildren.<Transform>();
	//Set up the length of all transform
	int_wayLength = waypoints.Length;
	int_wayIndex = 0;
	int_nextIndex = 1;
	//Checking the orderDirection if it?s false means the AI isn?t moving by order, 
	//so using the random index of waypoint
	if(orderDirection == false) {
		var int_randomWay : int = Mathf.Floor(Random.value * int_wayLength);
		//Checking to make sure that the waypoint length is more than 1
		if (int_wayLength > 1) {
			//Use Random Index
			while (int_wayIndex == int_randomWay) {
				int_randomWay = Mathf.Floor(Random.value * int_wayLength);
			}
		}
		int_nextIndex = int_randomWay;
	}
	//Set the direction to zero
	v3_direction = Vector3.zero;
	//To ignore the first waypoint at the beginning of the game
	b_isHitRadius = true;
}

public function StartPosition() : Vector3 {
	return waypoints[0].position;
} 

//Return the direction of the enemy toward the next waypoint
public function GetDirection( _AI : Transform ) : Vector3 {
	if (Vector3.Distance(_AI.position, waypoints[int_nextIndex].position) <= radius) {
		//Only check once when the AI hit the way point
		if (!b_isHitRadius) {
			b_isHitRadius = true;
			//Update the current way index
			int_wayIndex = int_nextIndex;
			//Get Direction by order
			if (orderDirection == true) {
				//Get the next way index
				int_nextIndex = (int_nextIndex + 1) % int_wayLength;
		    } else {
		    	var int_randomWay : int = Mathf.Floor(Random.value * int_wayLength);
	    		//Checking to make sure that the waypoint length is more than 1
				if (int_wayLength > 1) {
					//Use Random Index
					while (int_wayIndex == int_randomWay) {
						int_randomWay = Mathf.Floor(Random.value * int_wayLength);
					}
				}
				int_nextIndex = int_randomWay;
	    	}
		}
	} else {
		b_isHitRadius = false;
	}

	//Get Direction from the current position of the character to the next way point
	//Make sure that the y position equal to the waypoint y position
	var v3_currentPosition : Vector3 = new Vector3(_AI.position.x, waypoints[int_nextIndex].position.y, _AI.position.z);
	v3_direction = (waypoints[int_nextIndex].position - v3_currentPosition).normalized;

	return v3_direction;
}

//To get the direction from current position of the enemy to the player
public function GetDirectionToPlayer ( _AI : Transform, _player : Transform ) : Vector3 {
	//Make sure that the y position equal to the waypoint y position
	var v3_currentPosition : Vector3 = new Vector3(_AI.position.x, waypoints[int_wayIndex].position.y, _AI.position.z);
	var v3_playerPosition : Vector3 = new Vector3(_player.position.x, waypoints[int_wayIndex].position.y, _player.position.z);
	v3_direction = (v3_playerPosition - v3_currentPosition).normalized;
	
	return v3_direction;
	
}

//Checking if the enemy is away from the target waypoint in the specific distance or not
public function AwayFromWaypoint (_AI : Transform, _distance : float) : boolean {
	if (Vector3.Distance(_AI.position, waypoints[int_nextIndex].position) >= _distance) {
		return true;
	} else {
		return false;
	}
}

//Draw Gizmos and Directional line
public function OnDrawGizmos() : void {
	//Get all Transform of this game objects include the children and the transform of this gameobject
	var waypointGizmos : Transform[] = gameObject.GetComponentsInChildren.<Transform>();
	if (waypointGizmos != null) {
		if (orderDirection == true) {
			//Draw line by the order of each waypoint 0,1,2,3,...
			for (var i : int = 0; i < waypointGizmos.Length; i++) {
				Gizmos.color = Color.red;
				//Get the next way point
				var n : int = (i + 1) % waypointGizmos.Length;
				Gizmos.DrawLine(waypointGizmos[i].position, waypointGizmos[n].position);
				Gizmos.DrawIcon(waypointGizmos[i].position, iconName);
				Gizmos.color = Color.green;
				Gizmos.DrawWireSphere(waypointGizmos[i].position, radius);
			}
		} else {
			//Draw line from one point to every points except itself
			for (var j : int = 0; j < waypointGizmos.Length; j++) {
				for (var k : int = j; k < waypointGizmos.Length; k++) {
					Gizmos.color = Color.red;
					Gizmos.DrawLine(waypointGizmos[j].position, waypointGizmos[k].position);
				}
				Gizmos.DrawIcon(waypointGizmos[j].position, iconName);
				Gizmos.color = Color.green;
				Gizmos.DrawWireSphere(waypointGizmos[j].position, radius);
			}
		}
	}
}
