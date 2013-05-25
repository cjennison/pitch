public var rocks : Rocks;

public function OnTriggerEnter(collider : Collider) : void {
	if ((collider.transform.tag == "Player") && (rocks.GetTrigger() == false)) {
		rocks.EnabledRigidbody();
		rocks.SetTrigger(true);
	}
}