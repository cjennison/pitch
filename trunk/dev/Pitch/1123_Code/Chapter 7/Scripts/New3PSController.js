public var robotRagdoll : GameObject;

//Character movement speed
public var runSpeed : int = 6;
public var walkSpeed : int = 2;
private var int_moveSpeed : int;
//Animation Params
public var _animation : Animation;
public var idleAnimation : AnimationClip;
public var walkAnimation : AnimationClip;
public var runAnimation : AnimationClip;
public var shotAnimation : AnimationClip;
public var walkAnimationSpeed : float = 1.5;
public var idleAnimationSpeed : float = 1.0;
public var runAnimationSpeed : float = 2.0;
public var shotAnimationSpeed : float = 0.5;

//Camera Rotation Limit
public var minRotateY : float = -15;
public var maxRotateY : float = 60;

//Scopr UI
public var scopeUI : GUITexture;
//Rocket Launcher
public var rocketLauncher : RocketLauncher;
//Shot Params
private var b_isPrepare : boolean = false;
private var b_isShot : boolean = false;

//Mouse Look
private var mouseLook : MouseLook_JS;
//Character Motor
private var motor : CharacterMotor;

//NEW
public var HP : float = 100;
private var MaxHP : float;

// Use this for initialization
public function Awake () : void {
	motor = GetComponent(CharacterMotor);
	Screen.showCursor = false;
	int_moveSpeed = walkSpeed;
	
	//Get MouseLook component
	mouseLook = Camera.main.GetComponent(MouseLook_JS);
	
	//Setup Animation
	_animation[walkAnimation.name].speed = walkAnimationSpeed;
	_animation[walkAnimation.name].wrapMode = WrapMode.Loop;
	_animation[runAnimation.name].speed = runAnimationSpeed;
	_animation[runAnimation.name].wrapMode = WrapMode.Loop;
	_animation[idleAnimation.name].speed = idleAnimationSpeed;
	_animation[idleAnimation.name].wrapMode = WrapMode.Loop;
	
	//NEW - HP
	MaxHP = HP;
}

//NEW - Hit by rocket
public function OnCollisionEnter(collision : Collision) : void {
	if (StaticVars.b_isGameOver == false) {
		if (collision.transform.tag == "Rocket") {
			var rocket : Rocket = collision.gameObject.GetComponent(Rocket);
			var f_damage : float = rocket.getDamage();
			HP -= f_damage;
			if (HP <= 0) {
				HP = 0;
				//NEW - add the ragdoll when the player died
				var obj_playerPrefab : GameObject = Instantiate(robotRagdoll, transform.position, transform.rotation);
				GameObject.Destroy(_animation.gameObject);
				GameObject.Destroy(rocketLauncher.gameObject);
				GameObject.Destroy(this);
				GameObject.Destroy(gameObject.GetComponent(CharacterMotor));
				GameObject.Destroy(gameObject.GetComponent(CharacterController));
				GameObject.Destroy(gameObject.GetComponent(MouseLook_JS));
				scopeUI.enabled = false;
				Screen.showCursor = true;
				var mainCamera : GameObject = GameObject.FindWithTag("MainCamera");
				GameObject.Destroy(mainCamera.GetComponent(MouseLook_JS));
				var v3_newLocalPosition : Vector3 = new Vector3(mainCamera.transform.localPosition.x, mainCamera.transform.localPosition.y, -3);
				mainCamera.transform.localPosition = Vector3.Slerp(mainCamera.transform.localPosition, v3_newLocalPosition, Time.time);
			}
		}
	}
}

//NEW - HP Percent
public function GetHpPercent() : float {
	return HP/MaxHP;
}

// Update is called once per frame
public function Update () : void {
	if (StaticVars.b_isGameOver == false) { // NEW - checking gameover
		// Get the input vector from kayboard or analog stick
		var directionVector = new Vector3(Input.GetAxis("Horizontal"), 0, Input.GetAxis("Vertical"));
		
		if (directionVector != Vector3.zero) {
			// Get the length of the directon vector and then normalize it
			// Dividing by the length is cheaper than normalizing when we already have the length anyway
			var directionLength = directionVector.magnitude;
			directionVector = directionVector / directionLength;
			
			// Make sure the length is no bigger than 1
			directionLength = Mathf.Min(1, directionLength);
			
			// Make the input vector more sensitive towards the extremes and less sensitive in the middle
			// This makes it easier to control slow speeds when using analog sticks
			directionLength = directionLength * directionLength;
			
			// Multiply the normalized direction vector by the modified length
			directionVector = directionVector * directionLength;
		}
		
		//Reload the rocket bullet
		if (Input.GetKey(KeyCode.R)) {
			BroadcastMessage("Reload");
		}
		
		//Aiming
		if (Input.GetKey(KeyCode.E)) {
			//Show the Scope UI
			scopeUI.enabled = true;
			//Set the maximum and minimum limit rotation on Y-axis for the main camera
			mouseLook.minimumY = minRotateY;
			mouseLook.maximumY = maxRotateY;
			//Checking if the character is playing the shot animation
			if (!b_isPrepare) {
				b_isShot = false;
				//Play the shot preparing animation function
				WaitForPrepare();
			} else {
				//If the player click fire play the shot animation again
				if ((Input.GetButton("Fire1")) && (!b_isShot)) {
					b_isShot = true;
					//Play the shot animation function
					WaitForShot();
				}
			}
			//No Movement Direction
			motor.inputMoveDirection = Vector3.zero;
		} else {
			//Hide the Scope UI
			scopeUI.enabled = false;
			//Set the prepare animation to false
			b_isPrepare = false;
			//No Y-axis Rotation
			mouseLook.minimumY = 0;
			mouseLook.maximumY = 0;
			//Change the movement speed of the character
			if (Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift)) {
				int_moveSpeed = runSpeed;
			} else {
				int_moveSpeed = walkSpeed;
			}
			motor.movement.maxForwardSpeed = int_moveSpeed;
			motor.movement.maxSidewaysSpeed = int_moveSpeed;
			motor.movement.maxBackwardsSpeed = int_moveSpeed;
			////////////////////////////////////////////////////////
			//Checking if the character is moving or not
			if (directionVector != Vector3.zero) {
				if (int_moveSpeed == walkSpeed) {
					_animation.CrossFade(walkAnimation.name);
				} else {
					_animation.CrossFade(runAnimation.name);
				}
			} else {
				_animation.CrossFade(idleAnimation.name);
			}
			// Apply the direction to the CharacterMotor
			motor.inputMoveDirection = transform.rotation * directionVector;
			motor.inputJump = Input.GetButton("Jump");
		}
	} else {  // NEW - checking gameover
		//Gameover
		Screen.showCursor = true;
		scopeUI.enabled = false;
		motor.inputMoveDirection = Vector3.zero;
		_animation.CrossFade(idleAnimation.name);
	}
}

private function WaitForShot () : IEnumerator {
	_animation[shotAnimation.name].speed = shotAnimationSpeed;
	_animation[shotAnimation.name].wrapMode = WrapMode.ClampForever;
	_animation.PlayQueued(shotAnimation.name, QueueMode.PlayNow);
	BroadcastMessage("Fire", shotAnimation.length); //to enable all the function name Fire in every MonoBehaviour Script
	
	yield WaitForSeconds (shotAnimation.length);
	b_isShot = false;
}

private function WaitForPrepare () : IEnumerator {
	_animation[shotAnimation.name].speed = shotAnimationSpeed * 2;
	_animation[shotAnimation.name].wrapMode = WrapMode.ClampForever;
	_animation.CrossFade(shotAnimation.name, 0.6);
	
	yield WaitForSeconds(shotAnimation.length);
	b_isPrepare = true;
}

// Require a character controller to be attached to the same game object
@script RequireComponent (CharacterMotor)
@script AddComponentMenu ("Character/New 3PS Controller")