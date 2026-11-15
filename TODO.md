# To-do list (shortcuts left for later)

Being fixed in the navigator step (step 9), so not listed below:
depth from a simulated pressure sensor, and the helm using only `/subN/nav/state`.

## Open

1. **Turning strength is unverified.** The helm's heading numbers were copied from
   Nemo; the wolfsub hull is longer and harder to turn. Test turning and driving
   forward, and add a way to command heading and speed.
2. **Not a proper ROS 2 package.** Programs are started with `python3 scripts/...`.
   Add `package.xml` and a launch file (one command that starts everything).
3. **Tests wait fixed times** (for example `sleep 12`) instead of checking that
   things are ready. They can fail on a slow start.
4. **"Missing position" safety is only tested by pausing the simulation.**
   Test it with a real loss of data while the sub is moving.
5. **Log files in git are overwritten** every time a test runs.
6. **World name is still `lrauv_demo`**, left over from Tethys. Renaming it also
   changes the sensor topic names in `config/bridge_template.yaml`.
7. **LLM start-up failure.** The first call while a model loads can return an
   error. Handle it (load the model first, retry) when the captain is built.
8. **Nemo's copied files are in this folder, uncommitted.** Move Nemo to its own
   git repo.
9. **The 3D window is untested.** All tests so far ran without it.
10. **Sonar is untested against a real object in this world**, and the camera
    picture content was never looked at.
11. **Behaviour at the surface is untested** (a stopped sub floats up to it).
12. **More than two subs is untested.**
13. **Convoy ship is unchanged from the first version**: it hovers at z = 0.3
    with gravity off and no collisions (planned for step 11).
14. **The IMU has no noise configured**, so its readings are perfect.
