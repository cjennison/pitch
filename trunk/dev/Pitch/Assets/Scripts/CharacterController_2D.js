//Public Vars - Displays in the inspector
public var f_speed:float = 5.0; //speed of pitch
public var loopSprites:SpriteManager[]; //controls the update of the sprite animation texture
public var jumpSprite:JumpSpriteManager;
public var layerMask:LayerMask;

//Private Vars - Unavailable in the inspector, hidden to other classes
private var in_direction:int;
private var b_isJumping:boolean;

private var f_height:float; //Height of the character
private var f_lastY:float; //Last Y position of the character

//Starts as soon as the game begins
public function Start():void {
	in_direction = 1;
	
	//Init the sprite manager
	for(var i:int = 0;i < loopSprites.length;i++){
		loopSprites[i].init();
	}
	
	mesh = GetComponent(MeshFilter).sharedMesh;
	f_height = mesh.bounds.size.y * transform.localScale.y;
	f_lastY = transform.position.y;
	b_isJumping = false;
	
	
	
	//Update Camera to Character Position
	/* Notes
	* transform - refers to the current script's host
	* transform.position - The object that references the objects position(x,y,z);
	
	*/
	Camera.main.transform.position = new Vector3(transform.position.x, transform.position.y, transform.position.z - 20);
}

public function Update():void {
	if(!b_isJumping){

		if(Input.GetButton("Horizontal")){
		//Walking
			in_direction = Input.GetAxis("Horizontal") < 0 ? -1 : 1;
			rigidbody.velocity =  new Vector3((in_direction*f_speed), rigidbody.velocity.y, 0);
			loopSprites[0].resetFrame();
			loopSprites[1].updateAnimation(in_direction, renderer.material);
		} else {
			//Stay
			loopSprites[1].resetFrame();
			loopSprites[0].updateAnimation(in_direction, renderer.material);
		}
		
		if(Input.GetButton("Jump")){
			b_isJumping = true;
			loopSprites[0].resetFrame();
			loopSprites[1].resetFrame();
			rigidbody.velocity = new Vector3(rigidbody.velocity.x, -Physics.gravity.y, 0);
		}
	} else {
		jumpSprite.updateJumpAnimation(in_direction, rigidbody.velocity.y, renderer.material);
	}
}

//Late update is called after the update function is run, without having to use a callback.
public function LateUpdate():void {
	var hit:RaycastHit; //define a raycast
	var v3_hit:Vector3 = transform.TransformDirection(-Vector3.up) * (f_height * 0.5);
	var v3_right : Vector3 = new Vector3(transform.position.x + (collider.bounds.size.x*0.45), transform.position.y, transform.position.z);
 	var v3_left : Vector3 = new Vector3(transform.position.x - (collider.bounds.size.x*0.45), transform.position.y, transform.position.z);
	
	if (Physics.Raycast (transform.position, v3_hit, hit, 2.5, layerMask.value)) { //if on ground
		b_isJumping = false;
    } else if (Physics.Raycast (v3_right, v3_hit, hit, 2.5, layerMask.value)) { //if hit on right
	  	  if (b_isJumping) {
	 			 b_isJumping = false;
	        }
    } else if (Physics.Raycast (v3_left, v3_hit, hit, 2.5, layerMask.value)) { //if hit on left
		if (b_isJumping) {
		  b_isJumping = false;
		        }
    } else {
	    if (!b_isJumping) { //otherwise, the character is in the air
		    if (Mathf.Floor(transform.position.y) == f_lastY) {
		      b_isJumping = false;
		    } else {
		      b_isJumping = true;
	   		}
      }
  }
f_lastY = Mathf.Floor(transform.position.y);
	
	//Update Camera
	Camera.main.transform.position = new Vector3(transform.position.x, transform.position.y, transform.position.z - 20); //Eventually to become a variable we can change
}


public function OnDrawGizmos() : void {
  mesh = GetComponent(MeshFilter).sharedMesh;
  f_height = mesh.bounds.size.y* transform.localScale.y;
  var v3_right : Vector3 = new Vector3(transform.position.x + (collider.bounds.size.x*0.45), transform.position.y, transform.position.z);
  var v3_left : Vector3 = new Vector3(transform.position.x - (collider.bounds.size.x*0.45), transform.position.y, transform.position.z);
  Gizmos.color = Color.red;
  Gizmos.DrawRay(transform.position, transform.TransformDirection (-Vector3.up) * (f_height * 0.5));
  Gizmos.DrawRay(v3_right, transform.TransformDirection (-Vector3.up) * (f_height * 0.5));
  Gizmos.DrawRay(v3_left, transform.TransformDirection (-Vector3.up) * (f_height * 0.5));
}


class SpriteManager {
	public var spriteTexture:Texture2D;
	public var in_framePerSec:int;
	public var in_gridX:int;
	public var in_gridY:int;
	
	private var f_timePercent:float;
	private var f_nextTime:float;
	private var f_gridX:float;
	private var f_gridY:float;
	private var in_curFrame:int;
	
	public function init():void {
		f_timePercent = 1.0/in_framePerSec;
		f_nextTime = f_timePercent;
		f_gridX = 1.0/in_gridX;
		f_gridY = 1.0/in_gridY;
		in_curFrame = 1;
	}
	
	public function updateAnimation (_direction:int, _material:Material):void {
		_material.mainTexture = spriteTexture;
		if(Time.time > f_nextTime){
			f_nextTime = Time.time + f_timePercent;
			in_curFrame++;
			if(in_curFrame > in_framePerSec){
				in_curFrame = 1;
			}
		}
		
		_material.mainTextureScale = new Vector2(_direction * f_gridX, f_gridY);
		var in_col:int = 0;
		if(in_gridY > 1){
			in_col = Mathf.Ceil(in_curFrame/in_gridX);
		}
		if(_direction == 1){ //right
			_material.mainTextureOffset = new Vector2(((in_curFrame)%in_gridX) * f_gridX, in_col*f_gridY);
		} else {
			_material.mainTextureOffset = new Vector2(((in_gridX + (in_curFrame)%in_gridX)) * f_gridX, in_col*f_gridY);
		}
		
		
	}
	
	public function resetFrame():void {
		in_curFrame = 1;
	}
}

class JumpSpriteManager {
  public var t_jumpStartTexture : Texture2D; //Alternative Jump Texture play after t_jumpReadyTextures
  public var t_jumpAirTexture : Texture2D; //Alternative Jump Texture play when the player in the air at the top position of projectile
  public var t_jumpDownTexture : Texture2D; //Alternative Jump Texture play when the player fall to the ground
  
  public function updateJumpAnimation (_direction : int, _velocityY : float, _material : Material) : void {
    //Checking for the player position in the air
    if ((_velocityY>= -2.0) && (_velocityY<= 2.0)) { //Top of the projectile
      _material.mainTexture =t_jumpAirTexture;
    } else if (_velocityY> 2.0) { //Start Jump
      _material.mainTexture = t_jumpStartTexture;
    } else {  //Fall
      _material.mainTexture = t_jumpDownTexture;
    }
    _material.mainTextureScale = new Vector2 (_direction * 1, 1);
    _material.mainTextureOffset = new Vector2 (_direction * 1, 1);
  }
}