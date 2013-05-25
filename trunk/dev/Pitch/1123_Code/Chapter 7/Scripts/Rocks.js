public var downForce : float = 10;
private var a_rigid : Rigidbody[]; //Array of the children's Rigidbody
private var b_isTrigger : boolean = false;	// Is this object is already triggered (Use for Trigger object)
private var in_count : int = 0; //Counting the number of Kinematic Rock

//Setup Index of Children before start
public function Awake () : void {
	b_isTrigger = false;
	a_childRock = new Array();
	int_childLength = 0;
	in_count = 0;
	//Get all children's rigidbody
	a_rigid = gameObject.GetComponentsInChildren.<Rigidbody>();
}

// Use this for initialization
public function Start () : void {
	//Disable rigidbody before it triggered or hit by rocket
	DisabledRigidBody();
}

//	Update every frame
public function Update () : void {
	if (b_isTrigger == true) {
		for (var r : Rigidbody in a_rigid) {
			if (r.isKinematic == false) {
				var f_sqrLen : float = (r.velocity).sqrMagnitude;
				if (f_sqrLen <= 0.0) {
					r.useGravity = false;
					r.isKinematic = true;
					in_count++;
				}
			}
		}
		//Stop updating if all the rocks stop moving
		if (in_count >= a_rigid.Length) {
			b_isTrigger = false;
		}
	}
}

public function GetTrigger() : boolean {
	return b_isTrigger;
}

public function SetTrigger( _isTrigger : boolean) : boolean {
	b_isTrigger = _isTrigger;
}

public function EnabledRigidbody () : void {
	for (var r : Rigidbody in a_rigid) {
		r.useGravity = true;
		r.isKinematic = false;
		//Apply the velocity to the rigidbody in the -y direction to make the object fall faster
		r.velocity = new Vector3(0, -downForce, 0);
	}
}

public function DisabledRigidBody() : void {
	for (var r : Rigidbody in a_rigid) {
		r.useGravity = false;
		r.isKinematic = true;
	}
}