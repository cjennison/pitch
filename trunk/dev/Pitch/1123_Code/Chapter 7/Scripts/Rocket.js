@script RequireComponent(ConstantForce)
//Add the explosion force and radius
public var explosionRadius : float = 50;
public var explosionForce : float = 1000;
public var timeOut : float = 3.0; // Destroy after 3.0 seconds.
public var explosionParticle : GameObject;

//NEW - set damage for this rocket
private var f_damage : float = 20;
//Set Damage
public function setDamage (_damage : float) : void {
	f_damage = _damage;
}
//Get Damage
public function getDamage () : float {
	return f_damage;
}

// Use this for initialization
public function Start () : void {
	Invoke("KillObject", timeOut);
}

public function OnCollisionEnter (others : Collision) : void {
	//Create the explosion on the first impact point of the rocket and collider
	var contactPoint : ContactPoint = others.contacts[0];
	var rotation : Quaternion = Quaternion.FromToRotation(Vector3.up, contactPoint.normal);
	GameObject.Instantiate(explosionParticle, contactPoint.point, rotation);

	//Get the transfrom position of the rocket
	var v3_position : Vector3 = transform.position;
	//Get all colliders that touches or insides the explosion radius
	var a_hits : Collider[] = Physics.OverlapSphere(v3_position, explosionRadius);
	for (var c : Collider in a_hits) {
		// Check tag
		if (c.tag == "Destructible") {
			//Get all rigidbodys of the colliders
			var r : Rigidbody = c.rigidbody;
			if (r != null) {
				//Explosion
				r.isKinematic = false;
				r.AddExplosionForce(explosionForce, v3_position, explosionRadius);
			}
		}
	}

	KillObject();
}

public function KillObject () : void {
	//Stop the emit the particle
	var emitter : ParticleEmitter = GetComponentInChildren(ParticleEmitter);
	if (emitter != null) {
		emitter.emit = false; // Stop Emit
	}
	
	//In here We set the particle to auto destruct to destroy itself after a life time (or we can setup it in the editor)
	var particleAnimator : ParticleAnimator = GetComponentInChildren(ParticleAnimator);
	if (particleAnimator != null) {
		particleAnimator.autodestruct = true;
	}
	
	//Detach the trail renderer in our praticles
	transform.DetachChildren();
	
	//Destroy this Object
	GameObject.Destroy(gameObject);
}
