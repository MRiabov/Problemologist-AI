2026-02-22T10:19:46.185395Z [info     ] SimulationLoop_init            filename=loop.py forbidden_sites=[] func_name=__init__ goal_sites=[] lineno=188
2026-02-22T10:19:46.186353Z [info     ] SimulationLoop_step_start      filename=loop.py func_name=step lineno=354 target_body_name=None
FAILED
tests/integration/test_physics_fluids_wp04.py::test_gpu_oom_retry_logic 2026-02-22T10:19:46.268733Z [warning  ] genesis_oom_detected           attempt=1 filename=genesis_backend.py func_name=load_scene lineno=204 reduction=0.75
PASSED
tests/integration/test_physics_fluids_wp04.py::test_electronics_fluid_damage_logic 2026-02-22T10:19:46.346308Z [warning  ] genesis_step_called_unbuilt_attempting_sync filename=genesis_backend.py func_name=step lineno=543
2026-02-22T10:19:46.348435Z [info     ] electronics_fluid_damage       filename=genesis_backend.py func_name=_check_electronics_fluid_damage lineno=656 part=controller
PASSED
tests/integration/test_physics_genesis.py::test_fluid_containment_integration 2026-02-22T10:19:46.426231Z [info     ] SimulationLoop_init            filename=loop.py forbidden_sites=[] func_name=__init__ goal_sites=[] lineno=188
2026-02-22T10:19:46.428313Z [info     ] SimulationLoop_step_start      filename=loop.py func_name=step lineno=354 target_body_name=bucket
PASSED
tests/integration/test_physics_genesis.py::test_fluid_containment_failure 2026-02-22T10:19:46.518197Z [info     ] SimulationLoop_init            filename=loop.py forbidden_sites=[] func_name=__init__ goal_sites=[] lineno=188
2026-02-22T10:19:46.518626Z [info     ] SimulationLoop_step_start      filename=loop.py func_name=step lineno=354 target_body_name=bucket
FAILED
tests/integration/test_physics_genesis.py::test_part_breakage_integration 2026-02-22T10:19:46.600523Z [info     ] SimulationLoop_init            filename=loop.py forbidden_sites=[] func_name=__init__ goal_sites=[] lineno=188
2026-02-22T10:19:46.600825Z [info     ] SimulationLoop_step_start      filename=loop.py func_name=step lineno=354 target_body_name=bucket
2026-02-22T10:19:46.601046Z [info     ] DEBUG_backend_failure          filename=loop.py func_name=_resolve_backend_failure lineno=633 reason=PART_BREAKAGE:world
PASSED
tests/integration/test_physics_parity.py::test_physics_parity_rigid_body 2026-02-22T10:19:46.674900Z [info     ] SimulationLoop_init            filename=loop.py forbidden_sites=[] func_name=__init__ goal_sites=[] lineno=188
2026-02-22T10:19:46.675139Z [info     ] SimulationLoop_step_start      filename=loop.py func_name=step lineno=354 target_body_name=None
2026-02-22T10:19:46.678820Z [info     ] loading_manufacturing_config   filename=config.py func_name=load_config lineno=28 path=/home/maksym/Work/proj/Problemologist/Problemologist-AI/worker_heavy/workbenches/manufacturing_config.yaml
2026-02-22T10:19:46.686997Z [info     ] manufacturing_config_loaded    filename=config.py func_name=load_config lineno=48 methods=['cnc', 'injection_molding', 'three_dp']
[Genesis] [10:19:46] [WARNING] Using a simulation timestep smaller than 2ms is not recommended for 'use_gjk_collision=False' as it could lead to numerically unstable collision detection.
Using a simulation timestep smaller than 2ms is not recommended for 'use_gjk_collision=False' as it could lead to numerically unstable collision detection.
[Genesis] [10:19:46] [INFO] Scene <bff950a> created.
Scene ~~~<<bff950a>>~~~ created.
[Genesis] [10:19:46] [INFO] Adding <gs.RigidEntity>. idx: 0, uid: <ffbb123>, morph: <gs.morphs.Plane>, material: <gs.materials.Rigid>.
Adding ~<<gs.RigidEntity>>~. idx: ~<0>~, uid: ~~~<<ffbb123>>~~~, morph: ~<<gs.morphs.Plane>>~, material: ~<<gs.materials.Rigid>>~.
[Genesis] [10:19:46] [INFO] Adding <gs.RigidEntity>. idx: 1, uid: <9ed9142>, morph: <gs.morphs.MJCF(file='/tmp/pytest-of-maksym/pytest-2/test_physics_parity_rigid_body0/scene.xml')>, material: <gs.materials.Rigid>.
Adding ~<<gs.RigidEntity>>~. idx: ~<1>~, uid: ~~~<<9ed9142>>~~~, morph: ~<<gs.morphs.MJCF(file='/tmp/pytest-of-maksym/pytest-2/test_physics_parity_rigid_body0/scene.xml')>>~, material: ~<<gs.materials.Rigid>>~.
[Genesis] [10:19:46] [INFO] Collision meshes are not visualized by default. To visualize them, please use `vis_mode='collision'` when calling `scene.add_entity`.
Collision meshes are not visualized by default. To visualize them, please use `vis_mode='collision'` when calling `scene.add_entity`.
2026-02-22T10:19:46.737601Z [info     ] genesis_building_scene         filename=genesis_backend.py func_name=_load_scene_internal lineno=527
[Genesis] [10:19:46] [INFO] Building scene <bff950a>...
Building scene ~~~<<bff950a>>~~~...
[Genesis] [10:19:47] [INFO] Compiling simulation kernels...
Compiling simulation kernels...
[Genesis] [10:19:48] [INFO] Building visualizer...
Building visualizer...
2026-02-22T10:19:48.336225Z [info     ] SimulationLoop_init            filename=loop.py forbidden_sites=[] func_name=__init__ goal_sites=[] lineno=188
2026-02-22T10:19:48.336399Z [info     ] SimulationLoop_step_start      filename=loop.py func_name=step lineno=354 target_body_name=None
[Genesis] [10:19:48] [INFO] Running at 179.13 FPS (179.13 FPS per env, 1 envs).
Running at ~<179.13>~ FPS (~<179.13>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:48] [INFO] Running at 198.56 FPS (198.56 FPS per env, 1 envs).
Running at ~<198.56>~ FPS (~<198.56>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:48] [INFO] Running at 198.02 FPS (198.02 FPS per env, 1 envs).
Running at ~<198.02>~ FPS (~<198.02>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:48] [INFO] Running at 178.09 FPS (178.09 FPS per env, 1 envs).
Running at ~<178.09>~ FPS (~<178.09>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:48] [INFO] Running at 197.25 FPS (197.25 FPS per env, 1 envs).
Running at ~<197.25>~ FPS (~<197.25>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:48] [INFO] Running at 196.59 FPS (196.59 FPS per env, 1 envs).
Running at ~<196.59>~ FPS (~<196.59>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:48] [INFO] Running at 196.05 FPS (196.05 FPS per env, 1 envs).
Running at ~<196.05>~ FPS (~<196.05>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:48] [INFO] Running at 195.39 FPS (195.39 FPS per env, 1 envs).
Running at ~<195.39>~ FPS (~<195.39>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:48] [INFO] Running at 194.99 FPS (194.99 FPS per env, 1 envs).
Running at ~<194.99>~ FPS (~<194.99>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:48] [INFO] Running at 194.70 FPS (194.70 FPS per env, 1 envs).
Running at ~<194.70>~ FPS (~<194.70>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:48] [INFO] Running at 194.22 FPS (194.22 FPS per env, 1 envs).
Running at ~<194.22>~ FPS (~<194.22>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:48] [INFO] Running at 193.78 FPS (193.78 FPS per env, 1 envs).
Running at ~<193.78>~ FPS (~<193.78>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:49] [INFO] Running at 193.44 FPS (193.44 FPS per env, 1 envs).
Running at ~<193.44>~ FPS (~<193.44>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:49] [INFO] Running at 192.91 FPS (192.91 FPS per env, 1 envs).
Running at ~<192.91>~ FPS (~<192.91>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:49] [INFO] Running at 173.80 FPS (173.80 FPS per env, 1 envs).
Running at ~<173.80>~ FPS (~<173.80>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:49] [INFO] Running at 173.51 FPS (173.51 FPS per env, 1 envs).
Running at ~<173.51>~ FPS (~<173.51>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:49] [INFO] Running at 192.18 FPS (192.18 FPS per env, 1 envs).
Running at ~<192.18>~ FPS (~<192.18>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:49] [INFO] Running at 191.75 FPS (191.75 FPS per env, 1 envs).
Running at ~<191.75>~ FPS (~<191.75>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:49] [INFO] Running at 191.22 FPS (191.22 FPS per env, 1 envs).
Running at ~<191.22>~ FPS (~<191.22>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:49] [INFO] Running at 190.83 FPS (190.83 FPS per env, 1 envs).
Running at ~<190.83>~ FPS (~<190.83>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:49] [INFO] Running at 190.50 FPS (190.50 FPS per env, 1 envs).
Running at ~<190.50>~ FPS (~<190.50>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:49] [INFO] Running at 171.48 FPS (171.48 FPS per env, 1 envs).
Running at ~<171.48>~ FPS (~<171.48>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:49] [INFO] Running at 171.56 FPS (171.56 FPS per env, 1 envs).
Running at ~<171.56>~ FPS (~<171.56>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:49] [INFO] Running at 171.88 FPS (171.88 FPS per env, 1 envs).
Running at ~<171.88>~ FPS (~<171.88>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:49] [INFO] Running at 190.60 FPS (190.60 FPS per env, 1 envs).
Running at ~<190.60>~ FPS (~<190.60>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:49] [INFO] Running at 190.12 FPS (190.12 FPS per env, 1 envs).
Running at ~<190.12>~ FPS (~<190.12>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:49] [INFO] Running at 189.67 FPS (189.67 FPS per env, 1 envs).
Running at ~<189.67>~ FPS (~<189.67>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:49] [INFO] Running at 189.28 FPS (189.28 FPS per env, 1 envs).
Running at ~<189.28>~ FPS (~<189.28>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:49] [INFO] Running at 170.79 FPS (170.79 FPS per env, 1 envs).
Running at ~<170.79>~ FPS (~<170.79>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:49] [INFO] Running at 171.11 FPS (171.11 FPS per env, 1 envs).
Running at ~<171.11>~ FPS (~<171.11>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:49] [INFO] Running at 171.44 FPS (171.44 FPS per env, 1 envs).
Running at ~<171.44>~ FPS (~<171.44>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:50] [INFO] Running at 190.02 FPS (190.02 FPS per env, 1 envs).
Running at ~<190.02>~ FPS (~<190.02>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:50] [INFO] Running at 189.36 FPS (189.36 FPS per env, 1 envs).
Running at ~<189.36>~ FPS (~<189.36>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:50] [INFO] Running at 170.60 FPS (170.60 FPS per env, 1 envs).
Running at ~<170.60>~ FPS (~<170.60>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:50] [INFO] Running at 189.15 FPS (189.15 FPS per env, 1 envs).
Running at ~<189.15>~ FPS (~<189.15>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:50] [INFO] Running at 188.73 FPS (188.73 FPS per env, 1 envs).
Running at ~<188.73>~ FPS (~<188.73>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:50] [INFO] Running at 188.42 FPS (188.42 FPS per env, 1 envs).
Running at ~<188.42>~ FPS (~<188.42>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:50] [INFO] Running at 170.03 FPS (170.03 FPS per env, 1 envs).
Running at ~<170.03>~ FPS (~<170.03>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:50] [INFO] Running at 188.55 FPS (188.55 FPS per env, 1 envs).
Running at ~<188.55>~ FPS (~<188.55>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:50] [INFO] Running at 170.13 FPS (170.13 FPS per env, 1 envs).
Running at ~<170.13>~ FPS (~<170.13>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:50] [INFO] Running at 169.67 FPS (169.67 FPS per env, 1 envs).
Running at ~<169.67>~ FPS (~<169.67>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:50] [INFO] Running at 132.15 FPS (132.15 FPS per env, 1 envs).
Running at ~<132.15>~ FPS (~<132.15>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:50] [INFO] Running at 169.65 FPS (169.65 FPS per env, 1 envs).
Running at ~<169.65>~ FPS (~<169.65>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:50] [INFO] Running at 169.60 FPS (169.60 FPS per env, 1 envs).
Running at ~<169.60>~ FPS (~<169.60>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:50] [INFO] Running at 150.90 FPS (150.90 FPS per env, 1 envs).
Running at ~<150.90>~ FPS (~<150.90>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:50] [INFO] Running at 169.79 FPS (169.79 FPS per env, 1 envs).
Running at ~<169.79>~ FPS (~<169.79>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:50] [INFO] Running at 170.07 FPS (170.07 FPS per env, 1 envs).
Running at ~<170.07>~ FPS (~<170.07>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:50] [INFO] Running at 170.36 FPS (170.36 FPS per env, 1 envs).
Running at ~<170.36>~ FPS (~<170.36>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:50] [INFO] Running at 170.55 FPS (170.55 FPS per env, 1 envs).
Running at ~<170.55>~ FPS (~<170.55>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:50] [INFO] Running at 170.86 FPS (170.86 FPS per env, 1 envs).
Running at ~<170.86>~ FPS (~<170.86>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:51] [INFO] Running at 171.10 FPS (171.10 FPS per env, 1 envs).
Running at ~<171.10>~ FPS (~<171.10>~ FPS per env, ~<1>~ envs).
[Genesis] [10:19:51] [INFO] Running at 171.36 FPS (171.36 FPS per env, 1 envs).
Running at ~<171.36>~ FPS (~<171.36>~ FPS per env, ~<1>~ envs).
PASSED
tests/integration/test_simulation_concurrency.py::test_simulation_concurrency_serialization FAILED
tests/integration/test_worker_concurrency.py::test_worker_concurrency FAILED
tests/models/test_schemas.py::TestBoundingBox::test_valid_bounding_box PASSED
tests/models/test_schemas.py::TestBoundingBox::test_list_to_tuple_coercion PASSED
tests/models/test_schemas.py::TestBoundingBox::test_invalid_coordinates_count PASSED
tests/models/test_schemas.py::TestReviewFrontmatter::test_approved_decision PASSED
tests/models/test_schemas.py::TestReviewFrontmatter::test_rejected_decision PASSED
tests/models/test_schemas.py::TestReviewFrontmatter::test_confirm_plan_refusal PASSED
tests/models/test_schemas.py::TestReviewFrontmatter::test_reject_plan_refusal PASSED
tests/models/test_schemas.py::TestReviewFrontmatter::test_invalid_decision PASSED
tests/models/test_schemas.py::TestReviewFrontmatter::test_empty_comments_default PASSED
tests/models/test_schemas.py::TestMovingPart::test_motor_type PASSED
tests/models/test_schemas.py::TestMovingPart::test_passive_type PASSED
tests/models/test_schemas.py::TestObjectivesYaml::test_valid_full_schema PASSED
tests/models/test_schemas.py::TestObjectivesYaml::test_missing_constraints PASSED
tests/models/test_schemas.py::TestObjectivesYaml::test_missing_objectives_section PASSED
tests/observability/test_feedback.py::test_report_trace_feedback_success FAILED
tests/observability/test_feedback.py::test_report_trace_feedback_no_langfuse_id FAILED
tests/observability/test_langfuse.py::test_get_langfuse_client_returns_none_without_credentials PASSED
tests/observability/test_langfuse.py::test_get_langfuse_client_uses_configured_host PASSED
tests/observability/test_langfuse.py::test_get_langfuse_client_returns_none_in_integration_test PASSED
tests/observability/test_langfuse.py::test_calculate_and_report_automated_score FAILED
tests/observability/test_logging.py::TestTraceIdContext::test_set_and_get_trace_id PASSED
tests/observability/test_logging.py::TestTraceIdContext::test_get_trace_id_default PASSED
tests/observability/test_logging.py::TestTraceIdContext::test_clear_trace_id PASSED
tests/observability/test_logging.py::TestAddTraceIdProcessor::test_adds_trace_id_when_set PASSED
tests/observability/test_logging.py::TestAddTraceIdProcessor::test_does_not_add_when_not_set PASSED
tests/observability/test_logging.py::TestConfigureLogging::test_configure_logging_json_output {"key": "value", "event": "test_message", "timestamp": "2026-02-22T10:19:51.530012Z", "logger": "tests.observability.test_logging", "level": "info"}
PASSED
tests/observability/test_logging.py::TestConfigureLogging::test_configure_logging_with_log_level {"event": "should_not_appear", "timestamp": "2026-02-22T10:19:51.531184Z", "logger": "tests.observability.test_logging", "level": "info"}
{"event": "should_appear", "timestamp": "2026-02-22T10:19:51.531313Z", "logger": "tests.observability.test_logging", "level": "warning"}
PASSED
tests/observability/test_logging.py::TestGetLogger::test_get_logger_returns_bound_logger PASSED
tests/observability/test_logging.py::TestGetLogger::test_get_logger_without_name PASSED
tests/observability/test_persistence.py::test_get_db_url_standard PASSED
tests/observability/test_persistence.py::test_get_db_url_asyncpg_conversion PASSED
tests/observability/test_persistence.py::test_get_db_url_psycopg_conversion PASSED
tests/observability/test_persistence.py::test_get_db_url_missing PASSED
tests/observability/test_persistence.py::test_setup_persistence FAILED
tests/observability/test_storage.py::test_upload_download Found credentials in environment variables.
{"bucket": "test-bucket", "key": "test_file.txt", "path": "/tmp/pytest-of-maksym/pytest-2/test_upload_download0/test.txt", "content_type": "text/plain", "event": "file_uploaded", "timestamp": "2026-02-22T10:19:51.795883Z", "logger": "shared.observability.storage", "level": "info"}
{"bucket": "test-bucket", "key": "test_file.txt", "path": "/tmp/pytest-of-maksym/pytest-2/test_upload_download0/downloaded.txt", "event": "file_downloaded", "timestamp": "2026-02-22T10:19:51.804558Z", "logger": "shared.observability.storage", "level": "info"}
PASSED
tests/observability/test_storage.py::test_async_upload_download Found credentials in environment variables.
FAILED
tests/observability/test_storage.py::test_list_files Found credentials in environment variables.
{"bucket": "test-bucket", "key": "file1.txt", "path": "/tmp/pytest-of-maksym/pytest-2/test_list_files0/f1.txt", "content_type": "text/plain", "event": "file_uploaded", "timestamp": "2026-02-22T10:19:52.028210Z", "logger": "shared.observability.storage", "level": "info"}
{"bucket": "test-bucket", "key": "folder/file2.txt", "path": "/tmp/pytest-of-maksym/pytest-2/test_list_files0/f2.txt", "content_type": "text/plain", "event": "file_uploaded", "timestamp": "2026-02-22T10:19:52.033732Z", "logger": "shared.observability.storage", "level": "info"}
PASSED
tests/observability/test_storage.py::test_get_presigned_url Found credentials in environment variables.
{"bucket": "test-bucket", "key": "test_file.txt", "path": "/tmp/pytest-of-maksym/pytest-2/test_get_presigned_url0/data.txt", "content_type": "text/plain", "event": "file_uploaded", "timestamp": "2026-02-22T10:19:52.169206Z", "logger": "shared.observability.storage", "level": "info"}
PASSED
tests/observability/test_tracing.py::test_get_trace_callback_missing_env PASSED
tests/observability/test_tracing.py::test_get_trace_callback_with_env PASSED
tests/observability/test_tracing.py::test_get_trace_callback_default_host PASSED
tests/observability/test_tracing_interaction.py::test_nodes_call_get_langfuse_callback FAILED
tests/observability/test_tracing_interaction.py::test_graph_initializes_langfuse_callback FAILED
tests/ops/test_backup.py::test_backup_postgres_success {"s3_bucket": "my-backup-bucket", "event": "Starting postgres backup", "timestamp": "2026-02-22T10:19:52.229002Z", "logger": "shared.ops.backup", "level": "info"}
{"s3_key": "backups/postgres/db_dump_20260222_101952.sql.gz", "event": "Postgres backup completed and uploaded", "timestamp": "2026-02-22T10:19:52.229969Z", "logger": "shared.ops.backup", "level": "info"}
PASSED
tests/ops/test_backup.py::test_backup_s3_files_success {"source": "source-bucket", "dest": "backup-bucket", "event": "Starting S3 sync", "timestamp": "2026-02-22T10:19:52.232228Z", "logger": "shared.ops.backup", "level": "info"}
{"source": "source-bucket", "dest": "backup-bucket", "file_count": 1, "event": "S3 sync completed", "timestamp": "2026-02-22T10:19:52.232731Z", "logger": "shared.ops.backup", "level": "info"}
PASSED
tests/ops/test_backup.py::test_backup_postgres_failure {"s3_bucket": "bucket", "event": "Starting postgres backup", "timestamp": "2026-02-22T10:19:52.233975Z", "logger": "shared.ops.backup", "level": "info"}
{"error": "Command 'pg_dump' returned non-zero exit status 1.", "event": "Postgres backup failed", "timestamp": "2026-02-22T10:19:52.234101Z", "logger": "shared.ops.backup", "level": "error"}
PASSED
tests/shared/test_circuit_builder_connector.py::test_connector_gap Circuit built successfully.
Elements: ['Vsupply']
Has connector element: False
PASSED
tests/shared/test_electronics_schemas.py::test_electronics_section_validation PASSED
tests/shared/test_electronics_schemas.py::test_assembly_definition_with_electronics PASSED
tests/shared/test_electronics_schemas.py::test_electronics_reference_validation 2026-02-22T10:19:52.242642Z [error    ] electronics_reference_error    error="Electronic component 'm1' references unknown part 'missing_part'" filename=file_validation.py func_name=validate_assembly_definition_yaml lineno=158
PASSED
tests/shared/test_events.py::test_emit_event PASSED
tests/shared/test_servo_motor.py::test_servo_motor_emission None context requested by Box
PASSED
tests/shared/test_simulation_schemas.py::test_stress_summary_validation PASSED
tests/shared/test_simulation_schemas.py::test_fluid_metric_validation PASSED
tests/shared/test_simulation_schemas.py::test_simulation_metrics_defaults PASSED
tests/shared/test_simulation_schemas.py::test_simulation_result_serialization PASSED
tests/shared/test_steerability.py::test_geometric_selection_valid PASSED
tests/shared/test_steerability.py::test_geometric_selection_invalid_vector PASSED
tests/shared/test_steerability.py::test_code_reference_valid PASSED
tests/shared/test_steerability.py::test_steerable_prompt_valid PASSED
tests/shared/test_steerability.py::test_steerable_prompt_defaults PASSED
tests/shared/test_view_utils.py::test_isometric_view_alignment PASSED
tests/shared/test_view_utils.py::test_isometric_view_orthogonal PASSED
tests/test_api_fuzzing.py::test_api_fuzzing[POST /topology/inspect] {"error": "Script not found at /tmp/pytest-of-maksym/pytest-2/test_api_fuzzing_POST__topolog0/script.py", "event": "api_inspect_topology_failed", "timestamp": "2026-02-22T10:19:52.282226Z", "logger": "worker_light.api.routes", "level": "error"}
{"error": "Script not found at /tmp/pytest-of-maksym/pytest-2/test_api_fuzzing_POST__topolog0/script.py", "event": "api_inspect_topology_failed", "timestamp": "2026-02-22T10:19:52.374393Z", "session_id": "fuzz-test-session", "logger": "worker_light.api.routes", "level": "error"}
{"error": "Script not found at /tmp/pytest-of-maksym/pytest-2/test_api_fuzzing_POST__topolog0/script.py", "event": "api_inspect_topology_failed", "timestamp": "2026-02-22T10:19:52.379724Z", "session_id": "fuzz-test-session", "logger": "worker_light.api.routes", "level": "error"}
{"error": "Script not found at /tmp/pytest-of-maksym/pytest-2/test_api_fuzzing_POST__topolog0/script.py", "event": "api_inspect_topology_failed", "timestamp": "2026-02-22T10:19:52.389164Z", "session_id": "fuzz-test-session", "logger": "worker_light.api.routes", "level": "error"}
{"error": "Script not found at /tmp/pytest-of-maksym/pytest-2/test_api_fuzzing_POST__topolog0/script.py", "event": "api_inspect_topology_failed", "timestamp": "2026-02-22T10:19:52.420934Z", "session_id": "fuzz-test-session", "logger": "worker_light.api.routes", "level": "error"}
{"error": "Script not found at /tmp/pytest-of-maksym/pytest-2/test_api_fuzzing_POST__topolog0/script.py", "event": "api_inspect_topology_failed", "timestamp": "2026-02-22T10:19:52.442859Z", "session_id": "fuzz-test-session", "logger": "worker_light.api.routes", "level": "error"}
{"error": "Script not found at /tmp/pytest-of-maksym/pytest-2/test_api_fuzzing_POST__topolog0/script.py", "event": "api_inspect_topology_failed", "timestamp": "2026-02-22T10:19:52.450099Z", "session_id": "fuzz-test-session", "logger": "worker_light.api.routes", "level": "error"}
{"error": "Script not found at /tmp/pytest-of-maksym/pytest-2/test_api_fuzzing_POST__topolog0/script.py", "event": "api_inspect_topology_failed", "timestamp": "2026-02-22T10:19:52.501392Z", "session_id": "fuzz-test-session", "logger": "worker_light.api.routes", "level": "error"}
{"error": "Script not found at /tmp/pytest-of-maksym/pytest-2/test_api_fuzzing_POST__topolog0/\u00ed`\u00d0\u00ff\u00d4C\u0096\u00cb", "event": "api_inspect_topology_failed", "timestamp": "2026-02-22T10:19:52.656036Z", "session_id": "fuzz-test-session", "logger": "worker_light.api.routes", "level": "error"}
{"error": "Script not found at /tmp/pytest-of-maksym/pytest-2/test_api_fuzzing_POST__topolog0/=", "event": "api_inspect_topology_failed", "timestamp": "2026-02-22T10:19:52.663682Z", "session_id": "fuzz-test-session", "logger": "worker_light.api.routes", "level": "error"}
{"error": "Script not found at /tmp/pytest-of-maksym/pytest-2/test_api_fuzzing_POST__topolog0/script.py", "event": "api_inspect_topology_failed", "timestamp": "2026-02-22T10:19:52.677949Z", "session_id": "fuzz-test-session", "logger": "worker_light.api.routes", "level": "error"}
{"error": "Script not found at /tmp/pytest-of-maksym/pytest-2/test_api_fuzzing_POST__topolog0/\u00d16", "event": "api_inspect_topology_failed", "timestamp": "2026-02-22T10:19:52.860090Z", "session_id": "fuzz-test-session", "logger": "worker_light.api.routes", "level": "error"}
{"error": "Script not found at /tmp/pytest-of-maksym/pytest-2/test_api_fuzzing_POST__topolog0/script.py", "event": "api_inspect_topology_failed", "timestamp": "2026-02-22T10:19:53.029758Z", "session_id": "fuzz-test-session", "logger": "worker_light.api.routes", "level": "error"}
{"error": "Script not found at /tmp/pytest-of-maksym/pytest-2/test_api_fuzzing_POST__topolog0/script.py", "event": "api_inspect_topology_failed", "timestamp": "2026-02-22T10:19:53.290852Z", "session_id": "fuzz-test-session", "logger": "worker_light.api.routes", "level": "error"}
{"error": "Script not found at /tmp/pytest-of-maksym/pytest-2/test_api_fuzzing_POST__topolog0/script.py", "event": "api_inspect_topology_failed", "timestamp": "2026-02-22T10:19:53.298925Z", "session_id": "fuzz-test-session", "logger": "worker_light.api.routes", "level": "error"}
{"error": "Script not found at /tmp/pytest-of-maksym/pytest-2/test_api_fuzzing_POST__topolog0/script.py", "event": "api_inspect_topology_failed", "timestamp": "2026-02-22T10:19:53.535777Z", "session_id": "fuzz-test-session", "logger": "worker_light.api.routes", "level": "error"}
{"error": "Script not found at /tmp/pytest-of-maksym/pytest-2/test_api_fuzzing_POST__topolog0/script.py", "event": "api_inspect_topology_failed", "timestamp": "2026-02-22T10:19:53.546004Z", "session_id": "fuzz-test-session", "logger": "worker_light.api.routes", "level": "error"}
{"error": "Script not found at /tmp/pytest-of-maksym/pytest-2/test_api_fuzzing_POST__topolog0/script.py", "event": "api_inspect_topology_failed", "timestamp": "2026-02-22T10:19:53.552172Z", "session_id": "fuzz-test-session", "logger": "worker_light.api.routes", "level": "error"}
{"error": "Script not found at /tmp/pytest-of-maksym/pytest-2/test_api_fuzzing_POST__topolog0/script.py", "event": "api_inspect_topology_failed", "timestamp": "2026-02-22T10:19:53.558771Z", "session_id": "fuzz-test-session", "logger": "worker_light.api.routes", "level": "error"}
{"error": "Script not found at /tmp/pytest-of-maksym/pytest-2/test_api_fuzzing_POST__topolog0/\u00015@\\\udb01\udfe1", "event": "api_inspect_topology_failed", "timestamp": "2026-02-22T10:19:53.573074Z", "session_id": "fuzz-test-session", "logger": "worker_light.api.routes", "level": "error"}
{"error": "Script not found at /tmp/pytest-of-maksym/pytest-2/test_api_fuzzing_POST__topolog0/script.py", "event": "api_inspect_topology_failed", "timestamp": "2026-02-22T10:19:53.581747Z", "session_id": "fuzz-test-session", "logger": "worker_light.api.routes", "level": "error"}
{"error": "Script not found at /tmp/pytest-of-maksym/pytest-2/test_api_fuzzing_POST__topolog0/script.py", "event": "api_inspect_topology_failed", "timestamp": "2026-02-22T10:19:53.589496Z", "session_id": "fuzz-test-session", "logger": "worker_light.api.routes", "level": "error"}
{"error": "Script not found at /tmp/pytest-of-maksym/pytest-2/test_api_fuzzing_POST__topolog0/\uc9a5?xRN)\u0091\u00e7`\u00d0\ud85f\udff9", "event": "api_inspect_topology_failed", "timestamp": "2026-02-22T10:19:53.613874Z", "session_id": "fuzz-test-session", "logger": "worker_light.api.routes", "level": "error"}
{"error": "Script not found at /tmp/pytest-of-maksym/pytest-2/test_api_fuzzing_POST__topolog0/\u00f2!", "event": "api_inspect_topology_failed", "timestamp": "2026-02-22T10:19:53.675206Z", "session_id": "fuzz-test-session", "logger": "worker_light.api.routes", "level": "error"}
{"error": "Script not found at /tmp/pytest-of-maksym/pytest-2/test_api_fuzzing_POST__topolog0/\u00f2!", "event": "api_inspect_topology_failed", "timestamp": "2026-02-22T10:19:53.689139Z", "session_id": "fuzz-test-session", "logger": "worker_light.api.routes", "level": "error"}
{"error": "Script not found at /tmp/pytest-of-maksym/pytest-2/test_api_fuzzing_POST__topolog0/\u00f2!", "event": "api_inspect_topology_failed", "timestamp": "2026-02-22T10:19:53.702843Z", "session_id": "fuzz-test-session", "logger": "worker_light.api.routes", "level": "error"}
{"error": "Script not found at /tmp/pytest-of-maksym/pytest-2/test_api_fuzzing_POST__topolog0/\u00f2!", "event": "api_inspect_topology_failed", "timestamp": "2026-02-22T10:19:53.717806Z", "session_id": "fuzz-test-session", "logger": "worker_light.api.routes", "level": "error"}
{"error": "Script not found at /tmp/pytest-of-maksym/pytest-2/test_api_fuzzing_POST__topolog0/\u00f2!", "event": "api_inspect_topology_failed", "timestamp": "2026-02-22T10:19:53.733893Z", "session_id": "fuzz-test-session", "logger": "worker_light.api.routes", "level": "error"}
{"error": "Script not found at /tmp/pytest-of-maksym/pytest-2/test_api_fuzzing_POST__topolog0/script.py", "event": "api_inspect_topology_failed", "timestamp": "2026-02-22T10:19:53.747493Z", "session_id": "fuzz-test-session", "logger": "worker_light.api.routes", "level": "error"}
{"error": "Script not found at /tmp/pytest-of-maksym/pytest-2/test_api_fuzzing_POST__topolog0/script.py", "event": "api_inspect_topology_failed", "timestamp": "2026-02-22T10:19:53.760078Z", "session_id": "fuzz-test-session", "logger": "worker_light.api.routes", "level": "error"}
{"error": "Script not found at /tmp/pytest-of-maksym/pytest-2/test_api_fuzzing_POST__topolog0/\u00b1\u00c7\u00c0", "event": "api_inspect_topology_failed", "timestamp": "2026-02-22T10:19:54.132047Z", "session_id": "fuzz-test-session", "logger": "worker_light.api.routes", "level": "error"}
{"error": "Script not found at /tmp/pytest-of-maksym/pytest-2/test_api_fuzzing_POST__topolog0/e", "event": "api_inspect_topology_failed", "timestamp": "2026-02-22T10:19:54.232844Z", "session_id": "fuzz-test-session", "logger": "worker_light.api.routes", "level": "error"}
{"error": "Script not found at /tmp/pytest-of-maksym/pytest-2/test_api_fuzzing_POST__topolog0/e", "event": "api_inspect_topology_failed", "timestamp": "2026-02-22T10:19:54.246161Z", "session_id": "fuzz-test-session", "logger": "worker_light.api.routes", "level": "error"}
{"error": "Script not found at /tmp/pytest-of-maksym/pytest-2/test_api_fuzzing_POST__topolog0/script.py", "event": "api_inspect_topology_failed", "timestamp": "2026-02-22T10:19:54.284687Z", "session_id": "fuzz-test-session", "logger": "worker_light.api.routes", "level": "error"}
{"error": "Script not found at /tmp/pytest-of-maksym/pytest-2/test_api_fuzzing_POST__topolog0/script.py", "event": "api_inspect_topology_failed", "timestamp": "2026-02-22T10:19:54.294169Z", "session_id": "fuzz-test-session", "logger": "worker_light.api.routes", "level": "error"}
{"error": "Script not found at /tmp/pytest-of-maksym/pytest-2/test_api_fuzzing_POST__topolog0/FALSE", "event": "api_inspect_topology_failed", "timestamp": "2026-02-22T10:19:54.307140Z", "session_id": "fuzz-test-session", "logger": "worker_light.api.routes", "level": "error"}
{"error": "Script not found at /tmp/pytest-of-maksym/pytest-2/test_api_fuzzing_POST__topolog0/\udb7c\udf47\u00c4\udabd\udf74X\u0011\u00a5\udaf8\udf78B", "event": "api_inspect_topology_failed", "timestamp": "2026-02-22T10:19:54.355964Z", "session_id": "fuzz-test-session", "logger": "worker_light.api.routes", "level": "error"}
{"error": "Script not found at /tmp/pytest-of-maksym/pytest-2/test_api_fuzzing_POST__topolog0/\udb7c\udf47\u00c4\udabd\udf74X\u0011\u00a5\udaf8\udf78B", "event": "api_inspect_topology_failed", "timestamp": "2026-02-22T10:19:54.369470Z", "session_id": "fuzz-test-session", "logger": "worker_light.api.routes", "level": "error"}
{"error": "Script not found at /tmp/pytest-of-maksym/pytest-2/test_api_fuzzing_POST__topolog0/script.py", "event": "api_inspect_topology_failed", "timestamp": "2026-02-22T10:19:54.412167Z", "session_id": "fuzz-test-session", "logger": "worker_light.api.routes", "level": "error"}
{"error": "Script not found at /tmp/pytest-of-maksym/pytest-2/test_api_fuzzing_POST__topolog0/script.py", "event": "api_inspect_topology_failed", "timestamp": "2026-02-22T10:19:54.426123Z", "session_id": "fuzz-test-session", "logger": "worker_light.api.routes", "level": "error"}
{"error": "Script not found at /tmp/pytest-of-maksym/pytest-2/test_api_fuzzing_POST__topolog0/\u00c6\u008f\u008d\t\u00fd\u001eY\ud8d8\udc51\ub328\ud9a9\udd8e", "event": "api_inspect_topology_failed", "timestamp": "2026-02-22T10:19:54.438633Z", "session_id": "fuzz-test-session", "logger": "worker_light.api.routes", "level": "error"}
{"error": "Script not found at /tmp/pytest-of-maksym/pytest-2/test_api_fuzzing_POST__topolog0/script.py", "event": "api_inspect_topology_failed", "timestamp": "2026-02-22T10:19:54.470185Z", "session_id": "fuzz-test-session", "logger": "worker_light.api.routes", "level": "error"}
{"error": "Script not found at /tmp/pytest-of-maksym/pytest-2/test_api_fuzzing_POST__topolog0/script.py", "event": "api_inspect_topology_failed", "timestamp": "2026-02-22T10:19:54.478588Z", "session_id": "fuzz-test-session", "logger": "worker_light.api.routes", "level": "error"}
{"error": "Script not found at /tmp/pytest-of-maksym/pytest-2/test_api_fuzzing_POST__topolog0/script.py", "event": "api_inspect_topology_failed", "timestamp": "2026-02-22T10:19:54.564706Z", "session_id": "fuzz-test-session", "logger": "worker_light.api.routes", "level": "error"}
{"error": "Script not found at /tmp/pytest-of-maksym/pytest-2/test_api_fuzzing_POST__topolog0/script.py", "event": "api_inspect_topology_failed", "timestamp": "2026-02-22T10:19:54.789401Z", "session_id": "fuzz-test-session", "logger": "worker_light.api.routes", "level": "error"}
{"error": "Script not found at /tmp/pytest-of-maksym/pytest-2/test_api_fuzzing_POST__topolog0/script.py", "event": "api_inspect_topology_failed", "timestamp": "2026-02-22T10:19:54.796802Z", "session_id": "fuzz-test-session", "logger": "worker_light.api.routes", "level": "error"}
PASSED
tests/test_api_fuzzing.py::test_api_fuzzing[POST /fs/ls] PASSED
tests/test_api_fuzzing.py::test_api_fuzzing[POST /fs/exists] PASSED
tests/test_api_fuzzing.py::test_api_fuzzing[POST /fs/read] PASSED
tests/test_api_fuzzing.py::test_api_fuzzing[POST /fs/write] PASSED
tests/test_api_fuzzing.py::test_api_fuzzing[POST /fs/edit] PASSED
tests/test_api_fuzzing.py::test_api_fuzzing[POST /fs/upload_file] Found invalid character 91 in header at 36
Found invalid character 13 in header at 72
Found invalid character 13 in header at 76
Found invalid character 13 in header at 77
Found invalid character 13 in header at 73
Found invalid character 91 in header at 36
Found invalid character 91 in header at 36
Found invalid character 91 in header at 36
Found invalid character 13 in header at 77
Found invalid character 13 in header at 77
Found invalid character 13 in header at 77
Found invalid character 91 in header at 36
Found invalid character 91 in header at 36
Found invalid character 91 in header at 36
Found invalid character 13 in header at 76
Found invalid character 13 in header at 72
Found invalid character 91 in header at 36
Found invalid character 91 in header at 36
Found invalid character 91 in header at 36
Found invalid character 91 in header at 36
Found invalid character 91 in header at 36
Found invalid character 91 in header at 36
Found invalid character 13 in header at 76
Found invalid character 91 in header at 36
Found invalid character 13 in header at 76
Found invalid character 91 in header at 36
Found invalid character 91 in header at 36
Found invalid character 91 in header at 36
Found invalid character 91 in header at 36
Found invalid character 91 in header at 36
PASSED
tests/test_api_fuzzing.py::test_api_fuzzing[POST /fs/read_blob] PASSED
tests/test_api_fuzzing.py::test_api_fuzzing[POST /fs/grep] PASSED
tests/test_api_fuzzing.py::test_api_fuzzing[POST /fs/bundle] PASSED
tests/test_api_fuzzing.py::test_api_fuzzing[POST /fs/delete] PASSED
tests/test_api_fuzzing.py::test_api_fuzzing[POST /git/init] PASSED
tests/test_api_fuzzing.py::test_api_fuzzing[POST /git/commit] PASSED
tests/test_api_fuzzing.py::test_api_fuzzing[GET /git/status] PASSED
tests/test_api_fuzzing.py::test_api_fuzzing[POST /git/resolve] PASSED
tests/test_api_fuzzing.py::test_api_fuzzing[POST /git/merge/abort] PASSED
tests/test_api_fuzzing.py::test_api_fuzzing[POST /git/merge/complete] PASSED
tests/test_api_fuzzing.py::test_api_fuzzing[POST /runtime/execute] PASSED
tests/test_api_fuzzing.py::test_api_fuzzing[GET /assets/{path}] PASSED
tests/test_api_fuzzing.py::test_api_fuzzing[POST /lint] PASSED
tests/test_api_fuzzing.py::test_api_fuzzing[GET /health] PASSED
tests/test_api_fuzzing.py::test_api_fuzzing[GET /] PASSED
tests/test_controller_api_extended.py::test_read_root PASSED
tests/test_controller_api_extended.py::test_health_extended PASSED
tests/test_controller_api_extended.py::test_run_agent_db_error PASSED
tests/test_controller_api_extended.py::test_list_skills_empty PASSED
tests/test_controller_api_extended.py::test_list_skills_success PASSED
tests/test_controller_api_extended.py::test_trigger_backup_unauthorized {"event": "Unauthorized backup attempt", "timestamp": "2026-02-22T10:20:38.030697Z", "logger": "controller.api.ops", "level": "warning"}
PASSED
tests/test_controller_api_extended.py::test_trigger_backup_invalid_secret {"event": "Unauthorized backup attempt", "timestamp": "2026-02-22T10:20:38.034295Z", "logger": "controller.api.ops", "level": "warning"}
PASSED
tests/test_controller_api_extended.py::test_trigger_backup_no_temporal {"event": "Temporal client not found in app state", "timestamp": "2026-02-22T10:20:38.038069Z", "logger": "controller.api.ops", "level": "error"}
PASSED
tests/test_controller_api_extended.py::test_trigger_backup_success {"workflow_id": "backup-d80095f1", "event": "Starting backup workflow", "timestamp": "2026-02-22T10:20:38.043248Z", "logger": "controller.api.ops", "level": "info"}
PASSED
tests/test_controller_api_extended.py::test_websocket_manager FAILED
tests/test_controller_api_extended.py::test_websocket_broadcast_failure FAILED
tests/test_controller_persistence.py::test_asset_content_persistence ERROR
tests/test_controller_persistence.py::test_episode_relationships ERROR
tests/test_controller_tasks.py::test_execute_agent_task_success FAILED
tests/test_controller_tasks.py::test_execute_agent_task_without_langfuse_callback FAILED
tests/test_cots_foundation.py::test_cots_item_validation PASSED
tests/test_cots_foundation.py::test_cots_item_invalid_category PASSED
tests/test_cots_foundation.py::test_init_db PASSED
tests/test_cots_search.py::test_search_cots_catalog_success FAILED
tests/test_cots_search.py::test_search_cots_catalog_no_results FAILED
tests/test_cots_search.py::test_search_cots_catalog_all_args FAILED
tests/test_dashboard_api.py::test_health_check PASSED
tests/test_dashboard_api.py::test_run_agent_endpoint {"episode_id": "79db4380-dbc2-43be-9d38-aa48421aa41c", "event": "task_registered", "timestamp": "2026-02-22T10:20:38.208712Z", "logger": "controller.api.manager", "level": "info"}
PASSED
tests/test_env_config.py::test_env_files_consistency PASSED
tests/test_episodes_api.py::test_get_episode_not_found PASSED
tests/test_episodes_api.py::test_get_episode_success PASSED
tests/test_episodes_api.py::test_list_episodes PASSED
tests/test_episodes_api.py::test_list_episodes_pagination PASSED
tests/test_integration_docker.py::test_services_health FAILED
tests/test_integration_docker.py::test_controller_to_worker_agent_run FAILED
tests/test_interrupt.py::test_execute_agent_task_cancelled FAILED
tests/test_interrupt.py::test_interrupt_episode_success FAILED
tests/test_observability_events.py::test_simulation_instability_event PASSED
tests/test_observability_events.py::test_submission_validation_event PASSED
tests/test_observability_events.py::test_cost_weight_delta_event PASSED
tests/test_observability_events.py::test_library_usage_event PASSED
tests/test_observability_events.py::test_review_decision_event PASSED
tests/test_observability_events.py::test_simulation_failure_mode_enum PASSED
tests/test_observability_full.py::test_component_usage_event PASSED
tests/test_observability_full.py::test_tool_invocation_event PASSED
tests/test_observability_full.py::test_manufacturability_check_event PASSED
tests/test_observability_full.py::test_scene_validation_event PASSED
tests/test_observability_full.py::test_render_requests PASSED
tests/test_observability_full.py::test_simulation_events PASSED
tests/test_observability_full.py::test_cots_search_event PASSED
tests/test_observability_full.py::test_plan_submission_events PASSED
tests/test_observability_full.py::test_escalation_events PASSED
tests/test_observability_full.py::test_lint_failure_events PASSED
tests/test_observability_full.py::test_logic_failure_event PASSED
tests/test_observability_full.py::test_skill_events PASSED
tests/test_observability_full.py::test_tool_specific_events PASSED
tests/test_observability_full.py::test_simulation_instability PASSED
tests/test_observability_full.py::test_submission_validation PASSED
tests/test_observability_full.py::test_cost_weight_delta PASSED
tests/test_observability_full.py::test_library_usage PASSED
tests/test_observability_full.py::test_review_decision PASSED
tests/test_persistence_models.py::test_logging_configuration 2026-02-22T10:20:38.498708Z [info     ] test message                   extra_field=value filename=test_persistence_models.py func_name=test_logging_configuration lineno=12
PASSED
tests/test_persistence_models.py::test_models_instantiation PASSED
tests/test_persistence_models.py::test_db_setup FAILED
tests/test_streaming_assets.py::test_episode_broadcaster FAILED
tests/test_streaming_assets.py::test_get_asset_glb FAILED
tests/test_streaming_assets.py::test_get_asset_py FAILED
tests/test_streaming_assets.py::test_get_asset_syntax_error FAILED
tests/test_temporal_simulation.py::test_remote_fs_middleware_temporal FAILED
tests/unit/test_validation_utils.py::test_get_stress_report_with_advice PASSED
tests/unit/test_validation_utils.py::test_define_fluid_updates_yaml PASSED
tests/unit/test_validation_utils.py::test_set_soft_mesh 2026-02-22T10:20:38.674106Z [info     ] set_soft_mesh_enabled          fem_enabled=True filename=validation.py func_name=set_soft_mesh lineno=218 part_id=bracket
PASSED
tests/workbenches/test_cnc.py::test_cnc_machinable_part 2026-02-22T10:20:38.677922Z [info     ] loading_manufacturing_config   filename=config.py func_name=load_config lineno=28 path=/home/maksym/Work/proj/Problemologist/Problemologist-AI/worker_heavy/workbenches/manufacturing_config.yaml
2026-02-22T10:20:38.683703Z [info     ] manufacturing_config_loaded    filename=config.py func_name=load_config lineno=48 methods=['cnc', 'injection_molding', 'three_dp']
None context requested by None
Entering BuildPart with mode=Mode.ADD which is in different scope as parent
WorkplaneList is pushing 1 workplanes: [Plane(o=(0.00, 0.00, 0.00), x=(1.00, 0.00, 0.00), z=(0.00, 0.00, 1.00))]
LocationList is pushing 1 points: [(p=(0.00, 0.00, 0.00), o=(-0.00, 0.00, -0.00))]
BuildPart context requested by Box
Completed integrating 1 object(s) into part with Mode=Mode.ADD
BuildPart context requested by None
BuildPart context requested by None
BuildPart context requested by None
Entering BuildSketch with mode=Mode.ADD which is in same scope as parent
WorkplaneList is pushing 1 workplanes: [Plane(o=(0.00, 0.00, 5.00), x=(1.00, 0.00, 0.00), z=(0.00, 0.00, 1.00))]
LocationList is pushing 1 points: [(p=(0.00, 0.00, 0.00), o=(-0.00, 0.00, -0.00))]
BuildSketch context requested by Rectangle
Completed integrating 1 object(s) into part with Mode=Mode.ADD
LocationList is popping 1 points
WorkplaneList is popping 1 workplanes
Exiting BuildSketch
BuildPart context requested by extrude
1 face(s) to extrude on 1 face plane(s)
Completed integrating 1 object(s) into part with Mode=Mode.SUBTRACT
BuildPart context requested by fillet
Completed integrating 1 object(s) into part with Mode=Mode.REPLACE
LocationList is popping 1 points
WorkplaneList is popping 1 workplanes
Exiting BuildPart
2026-02-22T10:20:38.749775Z [info     ] starting_cnc_analysis          filename=cnc.py func_name=analyze_cnc lineno=219
2026-02-22T10:20:38.784316Z [info     ] calculating_cnc_cost           filename=cnc.py func_name=calculate_cnc_cost lineno=144 material=aluminum_6061 quantity=1
2026-02-22T10:20:38.792574Z [info     ] cnc_analysis_complete          filename=cnc.py func_name=analyze_cnc is_manufacturable=True lineno=262 violations=0
PASSED
tests/workbenches/test_cnc.py::test_cnc_undercut_part 2026-02-22T10:20:38.793732Z [info     ] loading_manufacturing_config   filename=config.py func_name=load_config lineno=28 path=/home/maksym/Work/proj/Problemologist/Problemologist-AI/worker_heavy/workbenches/manufacturing_config.yaml
2026-02-22T10:20:38.799416Z [info     ] manufacturing_config_loaded    filename=config.py func_name=load_config lineno=48 methods=['cnc', 'injection_molding', 'three_dp']
None context requested by None
Entering BuildPart with mode=Mode.ADD which is in different scope as parent
WorkplaneList is pushing 1 workplanes: [Plane(o=(0.00, 0.00, 0.00), x=(1.00, 0.00, 0.00), z=(0.00, 0.00, 1.00))]
LocationList is pushing 1 points: [(p=(0.00, 0.00, 0.00), o=(-0.00, 0.00, -0.00))]
BuildPart context requested by Box
Completed integrating 1 object(s) into part with Mode=Mode.ADD
BuildPart context requested by None
BuildPart context requested by None
BuildPart context requested by None
Entering BuildSketch with mode=Mode.ADD which is in same scope as parent
WorkplaneList is pushing 1 workplanes: [Plane(o=(10.00, 0.00, 0.00), x=(0.00, 0.00, 1.00), z=(1.00, 0.00, -0.00))]
LocationList is pushing 1 points: [(p=(0.00, 0.00, 0.00), o=(-0.00, 0.00, -0.00))]
BuildSketch context requested by Circle
Completed integrating 1 object(s) into part with Mode=Mode.ADD
LocationList is popping 1 points
WorkplaneList is popping 1 workplanes
Exiting BuildSketch
BuildPart context requested by extrude
1 face(s) to extrude on 1 face plane(s)
Completed integrating 1 object(s) into part with Mode=Mode.SUBTRACT
LocationList is popping 1 points
WorkplaneList is popping 1 workplanes
Exiting BuildPart
2026-02-22T10:20:38.819288Z [info     ] starting_cnc_analysis          filename=cnc.py func_name=analyze_cnc lineno=219
2026-02-22T10:20:38.828313Z [warning  ] cnc_undercuts_detected         count=376 filename=cnc.py func_name=analyze_cnc lineno=228
2026-02-22T10:20:38.834297Z [info     ] calculating_cnc_cost           filename=cnc.py func_name=calculate_cnc_cost lineno=144 material=aluminum_6061 quantity=1
2026-02-22T10:20:38.836496Z [info     ] cnc_analysis_complete          filename=cnc.py func_name=analyze_cnc is_manufacturable=False lineno=262 violations=1
None context requested by None
Entering BuildPart with mode=Mode.ADD which is in different scope as parent
WorkplaneList is pushing 1 workplanes: [Plane(o=(0.00, 0.00, 0.00), x=(1.00, 0.00, 0.00), z=(0.00, 0.00, 1.00))]
LocationList is pushing 1 points: [(p=(0.00, 0.00, 0.00), o=(-0.00, 0.00, -0.00))]
BuildPart context requested by Box
Completed integrating 1 object(s) into part with Mode=Mode.ADD
BuildPart context requested by None
BuildPart context requested by None
BuildPart context requested by None
Entering BuildPart with mode=Mode.ADD which is in same scope as parent
WorkplaneList is pushing 1 workplanes: [Plane(o=(0.00, 0.00, -2.50), x=(1.00, 0.00, 0.00), z=(-0.00, -0.00, -1.00))]
LocationList is pushing 1 points: [(p=(0.00, 0.00, 0.00), o=(-0.00, 0.00, -0.00))]
BuildPart context requested by Box
Completed integrating 1 object(s) into part with Mode=Mode.ADD
Completed integrating 1 object(s) into part with Mode=Mode.ADD
LocationList is popping 1 points
WorkplaneList is popping 1 workplanes
Exiting BuildPart
LocationList is popping 1 points
WorkplaneList is popping 1 workplanes
Exiting BuildPart
2026-02-22T10:20:38.859851Z [info     ] starting_cnc_analysis          filename=cnc.py func_name=analyze_cnc lineno=219
2026-02-22T10:20:38.864701Z [warning  ] cnc_undercuts_detected         count=16 filename=cnc.py func_name=analyze_cnc lineno=228
2026-02-22T10:20:38.884512Z [info     ] calculating_cnc_cost           filename=cnc.py func_name=calculate_cnc_cost lineno=144 material=aluminum_6061 quantity=1
2026-02-22T10:20:38.889686Z [info     ] cnc_analysis_complete          filename=cnc.py func_name=analyze_cnc is_manufacturable=False lineno=262 violations=1
PASSED
tests/workbenches/test_cnc.py::test_cnc_sharp_internal_corner 2026-02-22T10:20:38.892901Z [info     ] loading_manufacturing_config   filename=config.py func_name=load_config lineno=28 path=/home/maksym/Work/proj/Problemologist/Problemologist-AI/worker_heavy/workbenches/manufacturing_config.yaml
2026-02-22T10:20:38.903893Z [info     ] manufacturing_config_loaded    filename=config.py func_name=load_config lineno=48 methods=['cnc', 'injection_molding', 'three_dp']
None context requested by None
Entering BuildPart with mode=Mode.ADD which is in different scope as parent
WorkplaneList is pushing 1 workplanes: [Plane(o=(0.00, 0.00, 0.00), x=(1.00, 0.00, 0.00), z=(0.00, 0.00, 1.00))]
LocationList is pushing 1 points: [(p=(0.00, 0.00, 0.00), o=(-0.00, 0.00, -0.00))]
BuildPart context requested by Box
Completed integrating 1 object(s) into part with Mode=Mode.ADD
BuildPart context requested by None
BuildPart context requested by None
BuildPart context requested by None
Entering BuildSketch with mode=Mode.ADD which is in same scope as parent
WorkplaneList is pushing 1 workplanes: [Plane(o=(0.00, 0.00, 5.00), x=(1.00, 0.00, 0.00), z=(0.00, 0.00, 1.00))]
LocationList is pushing 1 points: [(p=(0.00, 0.00, 0.00), o=(-0.00, 0.00, -0.00))]
BuildSketch context requested by Rectangle
Completed integrating 1 object(s) into part with Mode=Mode.ADD
LocationList is popping 1 points
WorkplaneList is popping 1 workplanes
Exiting BuildSketch
BuildPart context requested by extrude
1 face(s) to extrude on 1 face plane(s)
Completed integrating 1 object(s) into part with Mode=Mode.SUBTRACT
LocationList is popping 1 points
WorkplaneList is popping 1 workplanes
Exiting BuildPart
2026-02-22T10:20:38.926580Z [info     ] starting_cnc_analysis          filename=cnc.py func_name=analyze_cnc lineno=219
2026-02-22T10:20:38.941341Z [info     ] calculating_cnc_cost           filename=cnc.py func_name=calculate_cnc_cost lineno=144 material=aluminum_6061 quantity=1
2026-02-22T10:20:38.944433Z [info     ] cnc_analysis_complete          filename=cnc.py func_name=analyze_cnc is_manufacturable=False lineno=262 violations=4
PASSED
tests/workbenches/test_cnc.py::test_cnc_cost_calculation 2026-02-22T10:20:38.945644Z [info     ] loading_manufacturing_config   filename=config.py func_name=load_config lineno=28 path=/home/maksym/Work/proj/Problemologist/Problemologist-AI/worker_heavy/workbenches/manufacturing_config.yaml
2026-02-22T10:20:38.951858Z [info     ] manufacturing_config_loaded    filename=config.py func_name=load_config lineno=48 methods=['cnc', 'injection_molding', 'three_dp']
None context requested by None
Entering BuildPart with mode=Mode.ADD which is in different scope as parent
WorkplaneList is pushing 1 workplanes: [Plane(o=(0.00, 0.00, 0.00), x=(1.00, 0.00, 0.00), z=(0.00, 0.00, 1.00))]
LocationList is pushing 1 points: [(p=(0.00, 0.00, 0.00), o=(-0.00, 0.00, -0.00))]
BuildPart context requested by Box
Completed integrating 1 object(s) into part with Mode=Mode.ADD
LocationList is popping 1 points
WorkplaneList is popping 1 workplanes
Exiting BuildPart
2026-02-22T10:20:38.955734Z [info     ] calculating_cnc_cost           filename=cnc.py func_name=calculate_cnc_cost lineno=144 material=aluminum_6061 quantity=1
2026-02-22T10:20:38.956726Z [info     ] calculating_cnc_cost           filename=cnc.py func_name=calculate_cnc_cost lineno=144 material=aluminum_6061 quantity=10
2026-02-22T10:20:38.957641Z [info     ] calculating_cnc_cost           filename=cnc.py func_name=calculate_cnc_cost lineno=144 material=aluminum_6061 quantity=1
2026-02-22T10:20:38.960681Z [info     ] calculating_cnc_cost           filename=cnc.py func_name=calculate_cnc_cost lineno=144 material=aluminum_6061 quantity=1
PASSED
tests/workbenches/test_config.py::test_load_config 2026-02-22T10:20:38.964816Z [info     ] loading_manufacturing_config   filename=config.py func_name=load_config lineno=28 path=/home/maksym/Work/proj/Problemologist/Problemologist-AI/worker_heavy/workbenches/manufacturing_config.yaml
2026-02-22T10:20:38.970698Z [info     ] manufacturing_config_loaded    filename=config.py func_name=load_config lineno=48 methods=['cnc', 'injection_molding', 'three_dp']
FAILED
tests/workbenches/test_config.py::test_config_file_not_found 2026-02-22T10:20:38.973622Z [info     ] loading_manufacturing_config   filename=config.py func_name=load_config lineno=28 path=non_existent_file.yaml
2026-02-22T10:20:38.973713Z [error    ] config_file_not_found          filename=config.py func_name=load_config lineno=31 path=non_existent_file.yaml
PASSED
tests/workbenches/test_config.py::test_invalid_yaml 2026-02-22T10:20:38.975114Z [info     ] loading_manufacturing_config   filename=config.py func_name=load_config lineno=28 path=/tmp/pytest-of-maksym/pytest-2/test_invalid_yaml0/invalid.yaml
2026-02-22T10:20:38.975413Z [error    ] config_load_failed             error='mapping values are not allowed here\n  in "/tmp/pytest-of-maksym/pytest-2/test_invalid_yaml0/invalid.yaml", line 1, column 14' filename=config.py func_name=load_config lineno=38
PASSED
tests/workbenches/test_config.py::test_validation_error 2026-02-22T10:20:38.978014Z [info     ] loading_manufacturing_config   filename=config.py func_name=load_config lineno=28 path=/tmp/pytest-of-maksym/pytest-2/test_validation_error0/invalid_data.yaml
2026-02-22T10:20:38.978376Z [error    ] config_validation_failed       error="1 validation error for ManufacturingConfig\ncnc\n  Input should be a valid dictionary or instance of MethodConfig [type=model_type, input_value='not a dict', input_type=str]\n    For further information visit https://errors.pydantic.dev/2.12/v/model_type" filename=config.py func_name=load_config lineno=51
PASSED
tests/workbenches/test_facade.py::test_facade_cnc None context requested by Box
2026-02-22T10:20:38.980589Z [info     ] loading_manufacturing_config   filename=config.py func_name=load_config lineno=28 path=/home/maksym/Work/proj/Problemologist/Problemologist-AI/worker_heavy/workbenches/manufacturing_config.yaml
2026-02-22T10:20:38.986554Z [info     ] manufacturing_config_loaded    filename=config.py func_name=load_config lineno=48 methods=['cnc', 'injection_molding', 'three_dp']
2026-02-22T10:20:38.986645Z [info     ] starting_dfm_facade_analysis   fem_required=False filename=dfm.py func_name=validate_and_price lineno=116 method=cnc
2026-02-22T10:20:38.986713Z [info     ] starting_cnc_analysis          filename=cnc.py func_name=analyze_cnc lineno=219
2026-02-22T10:20:38.995046Z [info     ] calculating_cnc_cost           filename=cnc.py func_name=calculate_cnc_cost lineno=144 material=aluminum_6061 quantity=1
2026-02-22T10:20:38.996564Z [info     ] cnc_analysis_complete          filename=cnc.py func_name=analyze_cnc is_manufacturable=True lineno=262 violations=0
PASSED
tests/workbenches/test_facade.py::test_facade_im None context requested by Box
2026-02-22T10:20:38.998729Z [info     ] loading_manufacturing_config   filename=config.py func_name=load_config lineno=28 path=/home/maksym/Work/proj/Problemologist/Problemologist-AI/worker_heavy/workbenches/manufacturing_config.yaml
2026-02-22T10:20:39.004591Z [info     ] manufacturing_config_loaded    filename=config.py func_name=load_config lineno=48 methods=['cnc', 'injection_molding', 'three_dp']
2026-02-22T10:20:39.004677Z [info     ] starting_dfm_facade_analysis   fem_required=False filename=dfm.py func_name=validate_and_price lineno=116 method=im
2026-02-22T10:20:39.004753Z [info     ] starting_im_analysis           filename=injection_molding.py func_name=analyze_im lineno=238
2026-02-22T10:20:39.007019Z [warning  ] insufficient_draft_detected    count=8 filename=injection_molding.py func_name=_check_draft_angles lineno=62
2026-02-22T10:20:39.008241Z [warning  ] wall_too_thick                 count=8 filename=injection_molding.py func_name=_check_wall_thickness lineno=114 max_dist=9.9999
2026-02-22T10:20:39.008361Z [info     ] calculating_im_cost            filename=injection_molding.py func_name=_calculate_im_cost lineno=166 material=abs quantity=1
2026-02-22T10:20:39.009799Z [info     ] im_cost_calculated             filename=injection_molding.py func_name=_calculate_im_cost lineno=207 tooling=5001.4 total=5001.483853333333 unit=0.08385333333333335
2026-02-22T10:20:39.010768Z [info     ] im_analysis_complete           filename=injection_molding.py func_name=analyze_im is_manufacturable=False lineno=285 violations=2
PASSED
tests/workbenches/test_facade.py::test_facade_3dp None context requested by Box
2026-02-22T10:20:39.013131Z [info     ] loading_manufacturing_config   filename=config.py func_name=load_config lineno=28 path=/home/maksym/Work/proj/Problemologist/Problemologist-AI/worker_heavy/workbenches/manufacturing_config.yaml
2026-02-22T10:20:39.019078Z [info     ] manufacturing_config_loaded    filename=config.py func_name=load_config lineno=48 methods=['cnc', 'injection_molding', 'three_dp']
2026-02-22T10:20:39.019164Z [info     ] starting_dfm_facade_analysis   fem_required=False filename=dfm.py func_name=validate_and_price lineno=116 method=3dp
2026-02-22T10:20:39.019247Z [info     ] starting_3dp_analysis          filename=print_3d.py func_name=analyze_3dp lineno=125
2026-02-22T10:20:39.020747Z [info     ] calculating_3dp_cost           filename=print_3d.py func_name=calculate_3dp_cost lineno=62 material=abs quantity=1
2026-02-22T10:20:39.021604Z [info     ] 3dp_analysis_complete          filename=print_3d.py func_name=analyze_3dp is_manufacturable=True lineno=175 violations=0
PASSED
tests/workbenches/test_facade.py::test_facade_invalid_method None context requested by Box
2026-02-22T10:20:39.023579Z [info     ] loading_manufacturing_config   filename=config.py func_name=load_config lineno=28 path=/home/maksym/Work/proj/Problemologist/Problemologist-AI/worker_heavy/workbenches/manufacturing_config.yaml
2026-02-22T10:20:39.030039Z [info     ] manufacturing_config_loaded    filename=config.py func_name=load_config lineno=48 methods=['cnc', 'injection_molding', 'three_dp']
2026-02-22T10:20:39.030266Z [info     ] starting_dfm_facade_analysis   fem_required=False filename=dfm.py func_name=validate_and_price lineno=116 method=nonexistent
2026-02-22T10:20:39.030343Z [error    ] unsupported_manufacturing_method filename=dfm.py func_name=validate_and_price lineno=146 method=nonexistent
PASSED
tests/workbenches/test_im.py::test_analyze_im_basic_box None context requested by Box
2026-02-22T10:20:39.032628Z [info     ] loading_manufacturing_config   filename=config.py func_name=load_config lineno=28 path=/home/maksym/Work/proj/Problemologist/Problemologist-AI/worker_heavy/workbenches/manufacturing_config.yaml
2026-02-22T10:20:39.038681Z [info     ] manufacturing_config_loaded    filename=config.py func_name=load_config lineno=48 methods=['cnc', 'injection_molding', 'three_dp']
2026-02-22T10:20:39.038803Z [info     ] starting_im_analysis           filename=injection_molding.py func_name=analyze_im lineno=238
2026-02-22T10:20:39.041323Z [warning  ] insufficient_draft_detected    count=8 filename=injection_molding.py func_name=_check_draft_angles lineno=62
2026-02-22T10:20:39.042576Z [warning  ] wall_too_thick                 count=8 filename=injection_molding.py func_name=_check_wall_thickness lineno=114 max_dist=9.9999
2026-02-22T10:20:39.042679Z [info     ] calculating_im_cost            filename=injection_molding.py func_name=_calculate_im_cost lineno=166 material=abs quantity=1
2026-02-22T10:20:39.043745Z [info     ] im_cost_calculated             filename=injection_molding.py func_name=_calculate_im_cost lineno=207 tooling=5001.4 total=5001.483853333333 unit=0.08385333333333335
2026-02-22T10:20:39.044278Z [info     ] im_analysis_complete           filename=injection_molding.py func_name=analyze_im is_manufacturable=False lineno=285 violations=2
PASSED
tests/workbenches/test_im.py::test_analyze_im_undercut None context requested by Box
None context requested by Box
None context requested by Box
2026-02-22T10:20:39.068878Z [info     ] loading_manufacturing_config   filename=config.py func_name=load_config lineno=28 path=/home/maksym/Work/proj/Problemologist/Problemologist-AI/worker_heavy/workbenches/manufacturing_config.yaml
2026-02-22T10:20:39.074758Z [info     ] manufacturing_config_loaded    filename=config.py func_name=load_config lineno=48 methods=['cnc', 'injection_molding', 'three_dp']
2026-02-22T10:20:39.074856Z [info     ] starting_im_analysis           filename=injection_molding.py func_name=analyze_im lineno=238
2026-02-22T10:20:39.079654Z [warning  ] insufficient_draft_detected    count=32 filename=injection_molding.py func_name=_check_draft_angles lineno=62
2026-02-22T10:20:39.080613Z [warning  ] im_undercuts_detected          count=12 filename=injection_molding.py func_name=analyze_im lineno=258
2026-02-22T10:20:39.081008Z [warning  ] wall_too_thick                 count=28 filename=injection_molding.py func_name=_check_wall_thickness lineno=114 max_dist=9.9999
2026-02-22T10:20:39.081095Z [info     ] calculating_im_cost            filename=injection_molding.py func_name=_calculate_im_cost lineno=166 material=abs quantity=1
2026-02-22T10:20:39.083493Z [info     ] im_cost_calculated             filename=injection_molding.py func_name=_calculate_im_cost lineno=207 tooling=5003.8 total=5003.918226666667 unit=0.11822666666666667
2026-02-22T10:20:39.084503Z [info     ] im_analysis_complete           filename=injection_molding.py func_name=analyze_im is_manufacturable=False lineno=285 violations=3
PASSED
tests/workbenches/test_im.py::test_analyze_im_wall_thickness None context requested by Box
2026-02-22T10:20:39.086618Z [info     ] loading_manufacturing_config   filename=config.py func_name=load_config lineno=28 path=/home/maksym/Work/proj/Problemologist/Problemologist-AI/worker_heavy/workbenches/manufacturing_config.yaml
2026-02-22T10:20:39.092630Z [info     ] manufacturing_config_loaded    filename=config.py func_name=load_config lineno=48 methods=['cnc', 'injection_molding', 'three_dp']
2026-02-22T10:20:39.092965Z [info     ] starting_im_analysis           filename=injection_molding.py func_name=analyze_im lineno=238
2026-02-22T10:20:39.095896Z [warning  ] insufficient_draft_detected    count=8 filename=injection_molding.py func_name=_check_draft_angles lineno=62
2026-02-22T10:20:39.097132Z [warning  ] wall_too_thin                  count=4 filename=injection_molding.py func_name=_check_wall_thickness lineno=107 min_dist=0.09990000149011612
2026-02-22T10:20:39.097239Z [warning  ] wall_too_thick                 count=8 filename=injection_molding.py func_name=_check_wall_thickness lineno=114 max_dist=9.9999
2026-02-22T10:20:39.097312Z [info     ] calculating_im_cost            filename=injection_molding.py func_name=_calculate_im_cost lineno=166 material=abs quantity=1
2026-02-22T10:20:39.098339Z [info     ] im_cost_calculated             filename=injection_molding.py func_name=_calculate_im_cost lineno=207 tooling=5001.02 total=5001.103359333334 unit=0.08335933333333334
2026-02-22T10:20:39.098844Z [info     ] im_analysis_complete           filename=injection_molding.py func_name=analyze_im is_manufacturable=False lineno=285 violations=3
PASSED
tests/workbenches/test_im.py::test_im_logging None context requested by Box
2026-02-22T10:20:39.100886Z [info     ] loading_manufacturing_config   filename=config.py func_name=load_config lineno=28 path=/home/maksym/Work/proj/Problemologist/Problemologist-AI/worker_heavy/workbenches/manufacturing_config.yaml
2026-02-22T10:20:39.106710Z [info     ] manufacturing_config_loaded    filename=config.py func_name=load_config lineno=48 methods=['cnc', 'injection_molding', 'three_dp']
PASSED
tests/workbenches/test_im.py::test_im_cost_calculation None context requested by Box
2026-02-22T10:20:39.114951Z [info     ] loading_manufacturing_config   filename=config.py func_name=load_config lineno=28 path=/home/maksym/Work/proj/Problemologist/Problemologist-AI/worker_heavy/workbenches/manufacturing_config.yaml
2026-02-22T10:20:39.124457Z [info     ] manufacturing_config_loaded    filename=config.py func_name=load_config lineno=48 methods=['cnc', 'injection_molding', 'three_dp']
2026-02-22T10:20:39.124575Z [info     ] starting_im_analysis           filename=injection_molding.py func_name=analyze_im lineno=238
2026-02-22T10:20:39.127365Z [warning  ] insufficient_draft_detected    count=8 filename=injection_molding.py func_name=_check_draft_angles lineno=62
2026-02-22T10:20:39.128612Z [warning  ] wall_too_thick                 count=8 filename=injection_molding.py func_name=_check_wall_thickness lineno=114 max_dist=9.9999
2026-02-22T10:20:39.128716Z [info     ] calculating_im_cost            filename=injection_molding.py func_name=_calculate_im_cost lineno=166 material=abs quantity=1
2026-02-22T10:20:39.129834Z [info     ] im_cost_calculated             filename=injection_molding.py func_name=_calculate_im_cost lineno=207 tooling=5001.4 total=5001.483853333333 unit=0.08385333333333335
2026-02-22T10:20:39.130387Z [info     ] im_analysis_complete           filename=injection_molding.py func_name=analyze_im is_manufacturable=False lineno=285 violations=2
2026-02-22T10:20:39.130491Z [info     ] calculating_im_cost            filename=injection_molding.py func_name=_calculate_im_cost lineno=166 material=abs quantity=1
2026-02-22T10:20:39.133050Z [info     ] im_cost_calculated             filename=injection_molding.py func_name=_calculate_im_cost lineno=207 tooling=5001.4 total=5001.483853333333 unit=0.08385333333333335
2026-02-22T10:20:39.133196Z [info     ] calculating_im_cost            filename=injection_molding.py func_name=_calculate_im_cost lineno=166 material=abs quantity=1
2026-02-22T10:20:39.135355Z [info     ] im_cost_calculated             filename=injection_molding.py func_name=_calculate_im_cost lineno=207 tooling=500.14 total=500.2238533333333 unit=0.08385333333333335
PASSED
tests/workbenches/test_print_3d.py::test_3dp_valid_part 2026-02-22T10:20:39.136563Z [info     ] loading_manufacturing_config   filename=config.py func_name=load_config lineno=28 path=/home/maksym/Work/proj/Problemologist/Problemologist-AI/worker_heavy/workbenches/manufacturing_config.yaml
2026-02-22T10:20:39.142739Z [info     ] manufacturing_config_loaded    filename=config.py func_name=load_config lineno=48 methods=['cnc', 'injection_molding', 'three_dp']
None context requested by Box
2026-02-22T10:20:39.144723Z [info     ] starting_3dp_analysis          filename=print_3d.py func_name=analyze_3dp lineno=125
2026-02-22T10:20:39.146445Z [info     ] calculating_3dp_cost           filename=print_3d.py func_name=calculate_3dp_cost lineno=62 material=abs quantity=1
2026-02-22T10:20:39.147405Z [info     ] 3dp_analysis_complete          filename=print_3d.py func_name=analyze_3dp is_manufacturable=True lineno=175 violations=0
FAILED
tests/workbenches/test_print_3d.py::test_3dp_invalid_part_multi_body 2026-02-22T10:20:39.151065Z [info     ] loading_manufacturing_config   filename=config.py func_name=load_config lineno=28 path=/home/maksym/Work/proj/Problemologist/Problemologist-AI/worker_heavy/workbenches/manufacturing_config.yaml
2026-02-22T10:20:39.156822Z [info     ] manufacturing_config_loaded    filename=config.py func_name=load_config lineno=48 methods=['cnc', 'injection_molding', 'three_dp']
None context requested by Box
None context requested by Box
2026-02-22T10:20:39.165283Z [info     ] starting_3dp_analysis          filename=print_3d.py func_name=analyze_3dp lineno=125
2026-02-22T10:20:39.168266Z [info     ] calculating_3dp_cost           filename=print_3d.py func_name=calculate_3dp_cost lineno=62 material=abs quantity=1
2026-02-22T10:20:39.169879Z [info     ] 3dp_analysis_complete          filename=print_3d.py func_name=analyze_3dp is_manufacturable=False lineno=175 violations=1
PASSED
tests/workbenches/test_print_3d.py::test_3dp_cost_calculation 2026-02-22T10:20:39.170955Z [info     ] loading_manufacturing_config   filename=config.py func_name=load_config lineno=28 path=/home/maksym/Work/proj/Problemologist/Problemologist-AI/worker_heavy/workbenches/manufacturing_config.yaml
2026-02-22T10:20:39.177762Z [info     ] manufacturing_config_loaded    filename=config.py func_name=load_config lineno=48 methods=['cnc', 'injection_molding', 'three_dp']
None context requested by Box
2026-02-22T10:20:39.179152Z [info     ] calculating_3dp_cost           filename=print_3d.py func_name=calculate_3dp_cost lineno=62 material=abs quantity=1
2026-02-22T10:20:39.179768Z [info     ] calculating_3dp_cost           filename=print_3d.py func_name=calculate_3dp_cost lineno=62 material=abs quantity=10
PASSED
tests/workbenches/test_print_3d.py::test_3dp_reuse_discount 2026-02-22T10:20:39.181221Z [info     ] loading_manufacturing_config   filename=config.py func_name=load_config lineno=28 path=/home/maksym/Work/proj/Problemologist/Problemologist-AI/worker_heavy/workbenches/manufacturing_config.yaml
2026-02-22T10:20:39.186905Z [info     ] manufacturing_config_loaded    filename=config.py func_name=load_config lineno=48 methods=['cnc', 'injection_molding', 'three_dp']
None context requested by Box
2026-02-22T10:20:39.188046Z [info     ] calculating_3dp_cost           filename=print_3d.py func_name=calculate_3dp_cost lineno=62 material=abs quantity=1
2026-02-22T10:20:39.190039Z [info     ] calculating_3dp_cost           filename=print_3d.py func_name=calculate_3dp_cost lineno=62 material=abs quantity=1
PASSED
tests/worker_heavy/simulation/test_builder.py::test_mesh_processor_dual_export None context requested by Box
PASSED
tests/worker_heavy/simulation/test_builder.py::test_scene_compiler Adding free joint to body 'box_body'. This part will fall if not supported. Use 'fixed=True' or 'constraint' attribute to secure it.
PASSED
tests/worker_heavy/simulation/test_builder.py::test_simulation_builder None context requested by Box
None context requested by Box
Adding free joint to body 'part_1'. This part will fall if not supported. Use 'fixed=True' or 'constraint' attribute to secure it.
PASSED
tests/worker_heavy/simulation/test_builder.py::test_vhacd_decomposition None context requested by Box
None context requested by Box
Adding free joint to body 'concave_part'. This part will fall if not supported. Use 'fixed=True' or 'constraint' attribute to secure it.
PASSED
tests/worker_heavy/simulation/test_builder.py::test_simulation_builder_missing_metadata_fails None context requested by Box
PASSED
tests/worker_heavy/simulation/test_builder_constraints.py::test_weld_constraint_generation None context requested by Box
None context requested by Box
Adding free joint to body 'base_part'. This part will fall if not supported. Use 'fixed=True' or 'constraint' attribute to secure it.
Adding free joint to body 'welded_part'. This part will fall if not supported. Use 'fixed=True' or 'constraint' attribute to secure it.
PASSED
tests/worker_heavy/simulation/test_builder_cots.py::test_add_actuator_cots_derivation PASSED
tests/worker_heavy/simulation/test_builder_cots.py::test_add_actuator_manual_override PASSED
tests/worker_heavy/simulation/test_builder_cots.py::test_add_actuator_unknown_motor PASSED
tests/worker_heavy/simulation/test_dynamic_control.py::test_dynamic_controllers 2026-02-22T10:20:39.837996Z [info     ] SimulationLoop_init            filename=loop.py forbidden_sites=[] func_name=__init__ goal_sites=[] lineno=188
2026-02-22T10:20:39.838534Z [info     ] SimulationLoop_step_start      filename=loop.py func_name=step lineno=354 target_body_name=None
FAILED
tests/worker_heavy/simulation/test_dynamic_control.py::test_dynamic_controllers_time_varying 2026-02-22T10:20:39.844960Z [info     ] SimulationLoop_init            filename=loop.py forbidden_sites=[] func_name=__init__ goal_sites=[] lineno=188
2026-02-22T10:20:39.845649Z [info     ] SimulationLoop_step_start      filename=loop.py func_name=step lineno=354 target_body_name=None
PASSED
tests/worker_heavy/simulation/test_factory.py::test_get_physics_backend PASSED
tests/worker_heavy/simulation/test_factory.py::test_get_physics_backend_invalid PASSED
tests/worker_heavy/simulation/test_factory.py::test_get_simulation_builder PASSED
tests/worker_heavy/simulation/test_genesis_builder_electronics.py::TestGenesisBuilderElectronics::test_applied_control_persistence PASSED
tests/worker_heavy/simulation/test_genesis_builder_electronics.py::TestGenesisBuilderElectronics::test_cables_in_scene_json PASSED
tests/worker_heavy/simulation/test_genesis_builder_electronics.py::TestGenesisBuilderElectronics::test_tendon_support PASSED
tests/worker_heavy/simulation/test_loop.py::test_initialization 2026-02-22T10:20:39.864236Z [info     ] SimulationLoop_init            filename=loop.py forbidden_sites=['zone_forbid_1'] func_name=__init__ goal_sites=['zone_goal'] lineno=188
PASSED
tests/worker_heavy/simulation/test_loop.py::test_step_simulation 2026-02-22T10:20:39.868277Z [info     ] SimulationLoop_init            filename=loop.py forbidden_sites=['zone_forbid_1'] func_name=__init__ goal_sites=['zone_goal'] lineno=188
2026-02-22T10:20:39.868814Z [info     ] SimulationLoop_step_start      filename=loop.py func_name=step lineno=354 target_body_name=target_box
PASSED
tests/worker_heavy/simulation/test_loop.py::test_metrics_collection 2026-02-22T10:20:39.872978Z [info     ] SimulationLoop_init            filename=loop.py forbidden_sites=['zone_forbid_1'] func_name=__init__ goal_sites=['zone_goal'] lineno=188
2026-02-22T10:20:39.873524Z [info     ] SimulationLoop_step_start      filename=loop.py func_name=step lineno=354 target_body_name=target_box
2026-02-22T10:20:39.875984Z [info     ] SimulationLoop_step_start      filename=loop.py func_name=step lineno=354 target_body_name=target_box
FAILED
tests/worker_heavy/simulation/test_loop.py::test_goal_zone_trigger 2026-02-22T10:20:39.884195Z [info     ] SimulationLoop_init            filename=loop.py forbidden_sites=['zone_forbid_1'] func_name=__init__ goal_sites=['zone_goal'] lineno=188
2026-02-22T10:20:39.884733Z [info     ] SimulationLoop_step_start      filename=loop.py func_name=step lineno=354 target_body_name=target_box
FAILED
tests/worker_heavy/simulation/test_loop.py::test_forbidden_zone_trigger 2026-02-22T10:20:39.891126Z [info     ] SimulationLoop_init            filename=loop.py forbidden_sites=['zone_forbid_1'] func_name=__init__ goal_sites=['zone_goal'] lineno=188
2026-02-22T10:20:39.891685Z [info     ] SimulationLoop_step_start      filename=loop.py func_name=step lineno=354 target_body_name=target_box
FAILED
tests/worker_heavy/simulation/test_loop.py::test_validation_hook_failure None context requested by Box
2026-02-22T10:20:39.898598Z [info     ] loading_manufacturing_config   filename=config.py func_name=load_config lineno=28 path=/home/maksym/Work/proj/Problemologist/Problemologist-AI/worker_heavy/workbenches/manufacturing_config.yaml
2026-02-22T10:20:39.904549Z [info     ] manufacturing_config_loaded    filename=config.py func_name=load_config lineno=48 methods=['cnc', 'injection_molding', 'three_dp']
[Genesis] [10:20:39] [WARNING] Using a simulation timestep smaller than 2ms is not recommended for 'use_gjk_collision=False' as it could lead to numerically unstable collision detection.
Using a simulation timestep smaller than 2ms is not recommended for 'use_gjk_collision=False' as it could lead to numerically unstable collision detection.
[Genesis] [10:20:39] [INFO] Scene <3fa23f4> created.
Scene ~~~<<3fa23f4>>~~~ created.
[Genesis] [10:20:39] [INFO] Adding <gs.RigidEntity>. idx: 0, uid: <ab91d2d>, morph: <gs.morphs.Plane>, material: <gs.materials.Rigid>.
Adding ~<<gs.RigidEntity>>~. idx: ~<0>~, uid: ~~~<<ab91d2d>>~~~, morph: ~<<gs.morphs.Plane>>~, material: ~<<gs.materials.Rigid>>~.
[Genesis] [10:20:39] [INFO] Adding <gs.RigidEntity>. idx: 1, uid: <8c3d951>, morph: <gs.morphs.MJCF(file='/tmp/pytest-of-maksym/pytest-2/test_validation_hook_failure0/test.xml')>, material: <gs.materials.Rigid>.
Adding ~<<gs.RigidEntity>>~. idx: ~<1>~, uid: ~~~<<8c3d951>>~~~, morph: ~<<gs.morphs.MJCF(file='/tmp/pytest-of-maksym/pytest-2/test_validation_hook_failure0/test.xml')>>~, material: ~<<gs.materials.Rigid>>~.
[Genesis] [10:20:39] [INFO] Collision meshes are not visualized by default. To visualize them, please use `vis_mode='collision'` when calling `scene.add_entity`.
Collision meshes are not visualized by default. To visualize them, please use `vis_mode='collision'` when calling `scene.add_entity`.
[Genesis] [10:20:39] [WARNING] Some free joint has non-zero frictionloss, damping or armature parameters. Beware it is non-physical.
Some free joint has non-zero frictionloss, damping or armature parameters. Beware it is non-physical.
2026-02-22T10:20:39.956115Z [info     ] genesis_building_scene         filename=genesis_backend.py func_name=_load_scene_internal lineno=527
[Genesis] [10:20:39] [INFO] Building scene <3fa23f4>...
Building scene ~~~<<3fa23f4>>~~~...
[Genesis] [10:20:40] [INFO] Compiling simulation kernels...
Compiling simulation kernels...
[Genesis] [10:20:40] [INFO] Building visualizer...
Building visualizer...
2026-02-22T10:20:40.820757Z [info     ] loading_manufacturing_config   filename=config.py func_name=load_config lineno=28 path=/home/maksym/Work/proj/Problemologist/Problemologist-AI/worker_heavy/workbenches/manufacturing_config.yaml
2026-02-22T10:20:40.826453Z [info     ] manufacturing_config_loaded    filename=config.py func_name=load_config lineno=48 methods=['cnc', 'injection_molding', 'three_dp']
2026-02-22T10:20:40.826591Z [info     ] loop_loaded_default_config     filename=loop.py func_name=__init__ lineno=133 materials=['aluminum_6061', 'abs', 'silicone_rubber']
2026-02-22T10:20:40.826715Z [info     ] SimulationLoop_init            filename=loop.py forbidden_sites=[] func_name=__init__ goal_sites=[] lineno=188
PASSED
tests/worker_heavy/simulation/test_loop.py::test_timeout_configurable 2026-02-22T10:20:40.830059Z [info     ] loading_manufacturing_config   filename=config.py func_name=load_config lineno=28 path=/home/maksym/Work/proj/Problemologist/Problemologist-AI/worker_heavy/workbenches/manufacturing_config.yaml
2026-02-22T10:20:40.835631Z [info     ] manufacturing_config_loaded    filename=config.py func_name=load_config lineno=48 methods=['cnc', 'injection_molding', 'three_dp']
[Genesis] [10:20:40] [WARNING] Using a simulation timestep smaller than 2ms is not recommended for 'use_gjk_collision=False' as it could lead to numerically unstable collision detection.
Using a simulation timestep smaller than 2ms is not recommended for 'use_gjk_collision=False' as it could lead to numerically unstable collision detection.
[Genesis] [10:20:40] [INFO] Scene <df261ad> created.
Scene ~~~<<df261ad>>~~~ created.
[Genesis] [10:20:40] [INFO] Adding <gs.RigidEntity>. idx: 0, uid: <4c7af16>, morph: <gs.morphs.Plane>, material: <gs.materials.Rigid>.
Adding ~<<gs.RigidEntity>>~. idx: ~<0>~, uid: ~~~<<4c7af16>>~~~, morph: ~<<gs.morphs.Plane>>~, material: ~<<gs.materials.Rigid>>~.
[Genesis] [10:20:40] [INFO] Adding <gs.RigidEntity>. idx: 1, uid: <ff9e7bf>, morph: <gs.morphs.MJCF(file='/tmp/pytest-of-maksym/pytest-2/test_timeout_configurable0/test.xml')>, material: <gs.materials.Rigid>.
Adding ~<<gs.RigidEntity>>~. idx: ~<1>~, uid: ~~~<<ff9e7bf>>~~~, morph: ~<<gs.morphs.MJCF(file='/tmp/pytest-of-maksym/pytest-2/test_timeout_configurable0/test.xml')>>~, material: ~<<gs.materials.Rigid>>~.
[Genesis] [10:20:40] [INFO] Collision meshes are not visualized by default. To visualize them, please use `vis_mode='collision'` when calling `scene.add_entity`.
Collision meshes are not visualized by default. To visualize them, please use `vis_mode='collision'` when calling `scene.add_entity`.
[Genesis] [10:20:40] [WARNING] Some free joint has non-zero frictionloss, damping or armature parameters. Beware it is non-physical.
Some free joint has non-zero frictionloss, damping or armature parameters. Beware it is non-physical.
2026-02-22T10:20:40.884005Z [info     ] genesis_building_scene         filename=genesis_backend.py func_name=_load_scene_internal lineno=527
[Genesis] [10:20:40] [INFO] Building scene <df261ad>...
Building scene ~~~<<df261ad>>~~~...
[Genesis] [10:20:41] [INFO] Compiling simulation kernels...
Compiling simulation kernels...
[Genesis] [10:20:41] [INFO] Building visualizer...
Building visualizer...
2026-02-22T10:20:41.727840Z [info     ] SimulationLoop_init            filename=loop.py forbidden_sites=[] func_name=__init__ goal_sites=[] lineno=188
2026-02-22T10:20:41.728262Z [info     ] SimulationLoop_step_start      filename=loop.py func_name=step lineno=354 target_body_name=target_box
[Genesis] [10:20:41] [INFO] Running at 162.26 FPS (162.26 FPS per env, 1 envs).
Running at ~<162.26>~ FPS (~<162.26>~ FPS per env, ~<1>~ envs).
[Genesis] [10:20:41] [INFO] Running at 162.65 FPS (162.65 FPS per env, 1 envs).
Running at ~<162.65>~ FPS (~<162.65>~ FPS per env, ~<1>~ envs).
2026-02-22T10:20:41.877539Z [info     ] timeout_triggered              filename=evaluator.py func_name=check_failure lineno=37 max_simulation_time=0.05 total_time=0.05000000000000003
PASSED
tests/worker_heavy/simulation/test_loop.py::test_timeout_capped_at_30s 2026-02-22T10:20:41.880689Z [info     ] loading_manufacturing_config   filename=config.py func_name=load_config lineno=28 path=/home/maksym/Work/proj/Problemologist/Problemologist-AI/worker_heavy/workbenches/manufacturing_config.yaml
2026-02-22T10:20:41.886590Z [info     ] manufacturing_config_loaded    filename=config.py func_name=load_config lineno=48 methods=['cnc', 'injection_molding', 'three_dp']
[Genesis] [10:20:41] [WARNING] Using a simulation timestep smaller than 2ms is not recommended for 'use_gjk_collision=False' as it could lead to numerically unstable collision detection.
Using a simulation timestep smaller than 2ms is not recommended for 'use_gjk_collision=False' as it could lead to numerically unstable collision detection.
[Genesis] [10:20:41] [INFO] Scene <7b32d6d> created.
Scene ~~~<<7b32d6d>>~~~ created.
[Genesis] [10:20:41] [INFO] Adding <gs.RigidEntity>. idx: 0, uid: <226a808>, morph: <gs.morphs.Plane>, material: <gs.materials.Rigid>.
Adding ~<<gs.RigidEntity>>~. idx: ~<0>~, uid: ~~~<<226a808>>~~~, morph: ~<<gs.morphs.Plane>>~, material: ~<<gs.materials.Rigid>>~.
[Genesis] [10:20:41] [INFO] Adding <gs.RigidEntity>. idx: 1, uid: <aee49b2>, morph: <gs.morphs.MJCF(file='/tmp/pytest-of-maksym/pytest-2/test_timeout_capped_at_30s0/test.xml')>, material: <gs.materials.Rigid>.
Adding ~<<gs.RigidEntity>>~. idx: ~<1>~, uid: ~~~<<aee49b2>>~~~, morph: ~<<gs.morphs.MJCF(file='/tmp/pytest-of-maksym/pytest-2/test_timeout_capped_at_30s0/test.xml')>>~, material: ~<<gs.materials.Rigid>>~.
[Genesis] [10:20:41] [INFO] Collision meshes are not visualized by default. To visualize them, please use `vis_mode='collision'` when calling `scene.add_entity`.
Collision meshes are not visualized by default. To visualize them, please use `vis_mode='collision'` when calling `scene.add_entity`.
[Genesis] [10:20:41] [WARNING] Some free joint has non-zero frictionloss, damping or armature parameters. Beware it is non-physical.
Some free joint has non-zero frictionloss, damping or armature parameters. Beware it is non-physical.
2026-02-22T10:20:41.932712Z [info     ] genesis_building_scene         filename=genesis_backend.py func_name=_load_scene_internal lineno=527
[Genesis] [10:20:41] [INFO] Building scene <7b32d6d>...
Building scene ~~~<<7b32d6d>>~~~...
[Genesis] [10:20:42] [INFO] Compiling simulation kernels...
Compiling simulation kernels...
[Genesis] [10:20:42] [INFO] Building visualizer...
Building visualizer...
2026-02-22T10:20:42.776647Z [info     ] SimulationLoop_init            filename=loop.py forbidden_sites=[] func_name=__init__ goal_sites=[] lineno=188
PASSED
tests/worker_heavy/simulation/test_loop.py::test_target_fell_off_world 2026-02-22T10:20:42.780703Z [info     ] SimulationLoop_init            filename=loop.py forbidden_sites=['zone_forbid_1'] func_name=__init__ goal_sites=['zone_goal'] lineno=188
2026-02-22T10:20:42.781165Z [info     ] SimulationLoop_step_start      filename=loop.py func_name=step lineno=354 target_body_name=target_box
FAILED
tests/worker_heavy/simulation/test_loop.py::test_instability_detection 2026-02-22T10:20:42.787510Z [info     ] SimulationLoop_init            filename=loop.py forbidden_sites=['zone_forbid_1'] func_name=__init__ goal_sites=['zone_goal'] lineno=188
2026-02-22T10:20:42.787955Z [info     ] SimulationLoop_step_start      filename=loop.py func_name=step lineno=354 target_body_name=target_box
FAILED
tests/worker_heavy/simulation/test_loop_stress.py::TestSimulationLoopStress::test_stress_collection 2026-02-22T10:20:42.795131Z [info     ] SimulationLoop_init            filename=loop.py forbidden_sites=[] func_name=__init__ goal_sites=[] lineno=188
2026-02-22T10:20:42.795669Z [info     ] SimulationLoop_step_start      filename=loop.py func_name=step lineno=354 target_body_name=target_box
FAILED
tests/worker_heavy/simulation/test_motor_overload.py::TestMotorOverload::test_overload_detection_triggers 2026-02-22T10:20:42.801187Z [info     ] SimulationLoop_init            filename=loop.py forbidden_sites=[] func_name=__init__ goal_sites=[] lineno=188
2026-02-22T10:20:42.801654Z [info     ] SimulationLoop_step_start      filename=loop.py func_name=step lineno=354 target_body_name=None
2026-02-22T10:20:42.805424Z [info     ] motor_overload_triggered       filename=evaluator.py force=0.1 func_name=check_motor_overload limit=0.1 lineno=86 motor=servo
FAILED
tests/worker_heavy/simulation/test_motor_overload.py::TestMotorOverload::test_normal_operation_no_overload 2026-02-22T10:20:42.811206Z [info     ] SimulationLoop_init            filename=loop.py forbidden_sites=[] func_name=__init__ goal_sites=[] lineno=188
2026-02-22T10:20:42.811817Z [info     ] SimulationLoop_step_start      filename=loop.py func_name=step lineno=354 target_body_name=None
WARNING: Nan, Inf or huge value in QACC at DOF 0. The simulation is unstable. Time = 1.0500.

PASSED
tests/worker_heavy/simulation/test_motor_overload.py::TestMotorOverload::test_overload_resets_on_unclamp 2026-02-22T10:20:42.818855Z [info     ] SimulationLoop_init            filename=loop.py forbidden_sites=[] func_name=__init__ goal_sites=[] lineno=188
PASSED
tests/worker_heavy/simulation/test_motor_overload.py::TestMotorOverload::test_threshold_constant_is_2_seconds PASSED
tests/worker_heavy/simulation/test_multi_run_verification.py::TestMultiRunVerification::test_default_num_runs PASSED
tests/worker_heavy/simulation/test_multi_run_verification.py::TestMultiRunVerification::test_consistent_success_all_pass 2026-02-22T10:20:42.824354Z [info     ] SimulationLoop_init            filename=loop.py forbidden_sites=[] func_name=__init__ goal_sites=[] lineno=188
2026-02-22T10:20:42.824605Z [info     ] SimulationLoop_step_start      filename=loop.py func_name=step lineno=354 target_body_name=target_box
2026-02-22T10:20:42.830666Z [info     ] SimulationLoop_init            filename=loop.py forbidden_sites=[] func_name=__init__ goal_sites=[] lineno=188
2026-02-22T10:20:42.831131Z [info     ] SimulationLoop_step_start      filename=loop.py func_name=step lineno=354 target_body_name=target_box
2026-02-22T10:20:42.836433Z [info     ] SimulationLoop_init            filename=loop.py forbidden_sites=[] func_name=__init__ goal_sites=[] lineno=188
2026-02-22T10:20:42.836821Z [info     ] SimulationLoop_step_start      filename=loop.py func_name=step lineno=354 target_body_name=target_box
PASSED
tests/worker_heavy/simulation/test_multi_run_verification.py::TestMultiRunVerification::test_deterministic_with_seed 2026-02-22T10:20:42.845061Z [info     ] SimulationLoop_init            filename=loop.py forbidden_sites=[] func_name=__init__ goal_sites=[] lineno=188
2026-02-22T10:20:42.845384Z [info     ] SimulationLoop_step_start      filename=loop.py func_name=step lineno=354 target_body_name=target_box
2026-02-22T10:20:42.849494Z [info     ] SimulationLoop_init            filename=loop.py forbidden_sites=[] func_name=__init__ goal_sites=[] lineno=188
2026-02-22T10:20:42.849924Z [info     ] SimulationLoop_step_start      filename=loop.py func_name=step lineno=354 target_body_name=target_box
2026-02-22T10:20:42.853276Z [info     ] SimulationLoop_init            filename=loop.py forbidden_sites=[] func_name=__init__ goal_sites=[] lineno=188
2026-02-22T10:20:42.853663Z [info     ] SimulationLoop_step_start      filename=loop.py func_name=step lineno=354 target_body_name=target_box
2026-02-22T10:20:42.857259Z [info     ] SimulationLoop_init            filename=loop.py forbidden_sites=[] func_name=__init__ goal_sites=[] lineno=188
2026-02-22T10:20:42.857469Z [info     ] SimulationLoop_step_start      filename=loop.py func_name=step lineno=354 target_body_name=target_box
2026-02-22T10:20:42.861634Z [info     ] SimulationLoop_init            filename=loop.py forbidden_sites=[] func_name=__init__ goal_sites=[] lineno=188
2026-02-22T10:20:42.862144Z [info     ] SimulationLoop_step_start      filename=loop.py func_name=step lineno=354 target_body_name=target_box
2026-02-22T10:20:42.865861Z [info     ] SimulationLoop_init            filename=loop.py forbidden_sites=[] func_name=__init__ goal_sites=[] lineno=188
2026-02-22T10:20:42.866298Z [info     ] SimulationLoop_step_start      filename=loop.py func_name=step lineno=354 target_body_name=target_box
PASSED
tests/worker_heavy/simulation/test_multi_run_verification.py::TestMultiRunVerification::test_result_has_metrics 2026-02-22T10:20:42.871655Z [info     ] SimulationLoop_init            filename=loop.py forbidden_sites=[] func_name=__init__ goal_sites=[] lineno=188
2026-02-22T10:20:42.871875Z [info     ] SimulationLoop_step_start      filename=loop.py func_name=step lineno=354 target_body_name=target_box
2026-02-22T10:20:42.875437Z [info     ] SimulationLoop_init            filename=loop.py forbidden_sites=[] func_name=__init__ goal_sites=[] lineno=188
2026-02-22T10:20:42.875878Z [info     ] SimulationLoop_step_start      filename=loop.py func_name=step lineno=354 target_body_name=target_box
PASSED
tests/worker_heavy/simulation/test_multi_run_verification.py::TestMultiRunVerification::test_jitter_range_default 2026-02-22T10:20:42.881883Z [info     ] SimulationLoop_init            filename=loop.py forbidden_sites=[] func_name=__init__ goal_sites=[] lineno=188
2026-02-22T10:20:42.882134Z [info     ] SimulationLoop_step_start      filename=loop.py func_name=step lineno=354 target_body_name=target_box
PASSED
tests/worker_heavy/simulation/test_position_controllers.py::TestWaypointController::test_waypoint_empty_schedule PASSED
tests/worker_heavy/simulation/test_position_controllers.py::TestWaypointController::test_waypoint_single_point PASSED
tests/worker_heavy/simulation/test_position_controllers.py::TestWaypointController::test_waypoint_multiple_points PASSED
tests/worker_heavy/simulation/test_position_controllers.py::TestWaypointController::test_waypoint_unsorted_input PASSED
tests/worker_heavy/simulation/test_position_controllers.py::TestHoldPosition::test_hold_position_constant PASSED
tests/worker_heavy/simulation/test_position_controllers.py::TestOscillate::test_oscillate_at_zero PASSED
tests/worker_heavy/simulation/test_position_controllers.py::TestOscillate::test_oscillate_at_quarter_period PASSED
tests/worker_heavy/simulation/test_position_controllers.py::TestOscillate::test_oscillate_with_center PASSED
tests/worker_heavy/simulation/test_position_controllers.py::TestPositionActuatorIntegration::test_hold_position_moves_joint 2026-02-22T10:20:42.892890Z [info     ] SimulationLoop_init            filename=loop.py forbidden_sites=[] func_name=__init__ goal_sites=[] lineno=188
2026-02-22T10:20:42.894160Z [info     ] SimulationLoop_step_start      filename=loop.py func_name=step lineno=354 target_body_name=None
PASSED
tests/worker_heavy/simulation/test_position_controllers.py::TestPositionActuatorIntegration::test_waypoint_sequence 2026-02-22T10:20:42.899475Z [info     ] SimulationLoop_init            filename=loop.py forbidden_sites=[] func_name=__init__ goal_sites=[] lineno=188
2026-02-22T10:20:42.899967Z [info     ] SimulationLoop_step_start      filename=loop.py func_name=step lineno=354 target_body_name=None
PASSED
tests/worker_heavy/simulation/test_randomization.py::TestGetEligibleMaterials::test_no_constraints_returns_all PASSED
tests/worker_heavy/simulation/test_randomization.py::TestGetEligibleMaterials::test_min_strength_filters PASSED
tests/worker_heavy/simulation/test_randomization.py::TestGetEligibleMaterials::test_whitelist_filters PASSED
tests/worker_heavy/simulation/test_randomization.py::TestGetEligibleMaterials::test_combined_filters PASSED
tests/worker_heavy/simulation/test_randomization.py::TestRandomizeMaterials::test_deterministic_with_seed PASSED
tests/worker_heavy/simulation/test_randomization.py::TestRandomizeMaterials::test_different_seeds_different_results PASSED
tests/worker_heavy/simulation/test_randomization.py::TestRandomizeMaterials::test_assignment_contains_correct_properties PASSED
tests/worker_heavy/simulation/test_randomization.py::TestRandomizeMaterials::test_raises_on_no_eligible PASSED
tests/worker_heavy/simulation/test_randomization.py::TestApplyMaterialToMjcfGeom::test_color_conversion PASSED
tests/worker_heavy/simulation/test_randomization.py::TestApplyMaterialToMjcfGeom::test_friction_output PASSED
tests/worker_heavy/simulation/test_randomization_xml.py::test_apply_randomization PASSED
tests/worker_heavy/simulation/test_weld_constraint.py::test_weld_constraint_generation None context requested by Box
None context requested by Box
Adding free joint to body 'part_1'. This part will fall if not supported. Use 'fixed=True' or 'constraint' attribute to secure it.
Adding free joint to body 'part_2'. This part will fall if not supported. Use 'fixed=True' or 'constraint' attribute to secure it.
PASSED
tests/worker_heavy/test_analyze_arguments.py::test_analyze_passes_method_and_quantity 2026-02-22T10:20:42.953268Z [info     ] loading_manufacturing_config   filename=config.py func_name=load_config lineno=28 path=/home/maksym/Work/proj/Problemologist/Problemologist-AI/worker_heavy/workbenches/manufacturing_config.yaml session_id=test-session
2026-02-22T10:20:42.959314Z [info     ] manufacturing_config_loaded    filename=config.py func_name=load_config lineno=48 methods=['cnc', 'injection_molding', 'three_dp'] session_id=test-session
PASSED
tests/worker_heavy/test_assembly_validation_regression.py::test_assembly_definition_validation_gap 2026-02-22T10:20:42.963668Z [info     ] plan_md_valid                  filename=file_validation.py func_name=validate_plan_md_structure lineno=352 plan_type=benchmark
2026-02-22T10:20:42.964646Z [error    ] cost_estimation_yaml_validation_error errors=["('constraints', 'benchmark_max_unit_cost_usd'): Input should be a valid number, unable to parse string as a number", "('constraints', 'planner_target_max_unit_cost_usd'): Field required", "('constraints', 'planner_target_max_weight_g'): Field required"] filename=file_validation.py func_name=validate_assembly_definition_yaml lineno=168
Validation result: False, Errors: ["assembly_definition.yaml: ('constraints', 'benchmark_max_unit_cost_usd'): Input should be a valid number, unable to parse string as a number", "assembly_definition.yaml: ('constraints', 'planner_target_max_unit_cost_usd'): Field required", "assembly_definition.yaml: ('constraints', 'planner_target_max_weight_g'): Field required"]
Found expected type error.
PASSED
tests/worker_heavy/test_benchmark_tools.py::test_benchmark_simulate_success PASSED
tests/worker_heavy/test_benchmark_tools.py::test_benchmark_validate_success PASSED
tests/worker_heavy/test_benchmark_tools.py::test_benchmark_simulate_error 2026-02-22T10:20:42.984481Z [error    ] api_benchmark_simulate_failed  error='Failed to simulate' filename=routes.py func_name=api_simulate lineno=241 session_id=test-session
PASSED
tests/worker_heavy/test_benchmark_tools.py::test_benchmark_analyze_success PASSED
tests/worker_heavy/test_dfm_material_logic.py::test_validate_and_price_uses_metadata_material None context requested by Box
2026-02-22T10:20:42.998861Z [info     ] starting_dfm_facade_analysis   fem_required=False filename=dfm.py func_name=validate_and_price lineno=116 method=cnc
2026-02-22T10:20:42.998961Z [info     ] starting_cnc_analysis          filename=cnc.py func_name=analyze_cnc lineno=219
2026-02-22T10:20:43.006222Z [info     ] calculating_cnc_cost           filename=cnc.py func_name=calculate_cnc_cost lineno=144 material=steel quantity=1
2026-02-22T10:20:43.007638Z [info     ] cnc_analysis_complete          filename=cnc.py func_name=analyze_cnc is_manufacturable=True lineno=262 violations=0
PASSED
tests/worker_heavy/test_dfm_material_logic.py::test_validate_and_price_fem_gate_missing_fields None context requested by Box
2026-02-22T10:20:43.010252Z [info     ] starting_dfm_facade_analysis   fem_required=True filename=dfm.py func_name=validate_and_price lineno=116 method=cnc
2026-02-22T10:20:43.010516Z [info     ] starting_cnc_analysis          filename=cnc.py func_name=analyze_cnc lineno=219
2026-02-22T10:20:43.018211Z [info     ] calculating_cnc_cost           filename=cnc.py func_name=calculate_cnc_cost lineno=144 material=aluminum_6061 quantity=1
2026-02-22T10:20:43.019612Z [info     ] cnc_analysis_complete          filename=cnc.py func_name=analyze_cnc is_manufacturable=True lineno=262 violations=0
PASSED
tests/worker_heavy/test_dfm_material_logic.py::test_validate_and_price_fem_gate_success None context requested by Box
2026-02-22T10:20:43.021703Z [info     ] starting_dfm_facade_analysis   fem_required=True filename=dfm.py func_name=validate_and_price lineno=116 method=cnc
2026-02-22T10:20:43.021786Z [info     ] starting_cnc_analysis          filename=cnc.py func_name=analyze_cnc lineno=219
2026-02-22T10:20:43.029971Z [info     ] calculating_cnc_cost           filename=cnc.py func_name=calculate_cnc_cost lineno=144 material=aluminum_6061 quantity=1
2026-02-22T10:20:43.031405Z [info     ] cnc_analysis_complete          filename=cnc.py func_name=analyze_cnc is_manufacturable=True lineno=262 violations=0
PASSED
tests/worker_heavy/test_dfm_material_logic.py::test_validate_and_price_no_metadata_uses_default None context requested by Box
2026-02-22T10:20:43.033508Z [info     ] starting_dfm_facade_analysis   fem_required=False filename=dfm.py func_name=validate_and_price lineno=116 method=cnc
2026-02-22T10:20:43.033591Z [info     ] starting_cnc_analysis          filename=cnc.py func_name=analyze_cnc lineno=219
2026-02-22T10:20:43.040680Z [info     ] calculating_cnc_cost           filename=cnc.py func_name=calculate_cnc_cost lineno=144 material=aluminum_6061 quantity=1
2026-02-22T10:20:43.042052Z [info     ] cnc_analysis_complete          filename=cnc.py func_name=analyze_cnc is_manufacturable=True lineno=262 violations=0
PASSED
tests/worker_heavy/test_dfm_material_logic.py::test_validate_and_price_empty_config_fallback None context requested by Box
2026-02-22T10:20:43.045202Z [info     ] starting_dfm_facade_analysis   fem_required=False filename=dfm.py func_name=validate_and_price lineno=116 method=cnc
2026-02-22T10:20:43.045318Z [info     ] starting_cnc_analysis          filename=cnc.py func_name=analyze_cnc lineno=219
2026-02-22T10:20:43.052881Z [info     ] calculating_cnc_cost           filename=cnc.py func_name=calculate_cnc_cost lineno=144 material=aluminum_6061 quantity=1
2026-02-22T10:20:43.054257Z [info     ] cnc_analysis_complete          filename=cnc.py func_name=analyze_cnc is_manufacturable=True lineno=262 violations=0
PASSED
tests/worker_heavy/test_file_validation.py::TestValidateObjectivesYaml::test_valid_content 2026-02-22T10:20:43.058448Z [info     ] objectives_yaml_valid          filename=file_validation.py func_name=validate_objectives_yaml lineno=93
PASSED
tests/worker_heavy/test_file_validation.py::TestValidateObjectivesYaml::test_empty_content PASSED
tests/worker_heavy/test_file_validation.py::TestValidateObjectivesYaml::test_missing_required_field 2026-02-22T10:20:43.062710Z [error    ] objectives_yaml_validation_error errors=["('simulation_bounds',): Field required", "('moved_object',): Field required", "('constraints',): Field required"] filename=file_validation.py func_name=validate_objectives_yaml lineno=100
PASSED
tests/worker_heavy/test_file_validation.py::TestValidateObjectivesYaml::test_invalid_yaml_syntax 2026-02-22T10:20:43.064459Z [error    ] objectives_yaml_parse_error    error='mapping values are not allowed here\n  in "<unicode string>", line 1, column 14:\n    invalid: yaml: content:\n                 ^' filename=file_validation.py func_name=validate_objectives_yaml lineno=96
PASSED
tests/worker_heavy/test_file_validation.py::TestValidateReviewFrontmatter::test_valid_approved 2026-02-22T10:20:43.065854Z [info     ] review_frontmatter_valid       decision=approved filename=file_validation.py func_name=validate_review_frontmatter lineno=220
PASSED
tests/worker_heavy/test_file_validation.py::TestValidateReviewFrontmatter::test_valid_rejected 2026-02-22T10:20:43.067103Z [info     ] review_frontmatter_valid       decision=rejected filename=file_validation.py func_name=validate_review_frontmatter lineno=220
PASSED
tests/worker_heavy/test_file_validation.py::TestValidateReviewFrontmatter::test_missing_frontmatter PASSED
tests/worker_heavy/test_file_validation.py::TestValidateReviewFrontmatter::test_invalid_decision_value 2026-02-22T10:20:43.069085Z [error    ] review_frontmatter_validation_error errors=["('decision',): Input should be 'approved', 'rejected', 'reject_plan', 'reject_code', 'confirm_plan_refusal' or 'reject_plan_refusal'"] filename=file_validation.py func_name=validate_review_frontmatter lineno=227
PASSED
tests/worker_heavy/test_file_validation.py::TestValidateReviewFrontmatter::test_refusal_decision_without_context PASSED
tests/worker_heavy/test_file_validation.py::TestValidateReviewFrontmatter::test_refusal_decision_with_context 2026-02-22T10:20:43.071331Z [info     ] review_frontmatter_valid       decision=confirm_plan_refusal filename=file_validation.py func_name=validate_review_frontmatter lineno=220
PASSED
tests/worker_heavy/test_file_validation.py::TestValidatePlanMdStructure::test_valid_benchmark_plan 2026-02-22T10:20:43.072274Z [info     ] plan_md_valid                  filename=file_validation.py func_name=validate_plan_md_structure lineno=352 plan_type=benchmark
PASSED
tests/worker_heavy/test_file_validation.py::TestValidatePlanMdStructure::test_missing_section_benchmark 2026-02-22T10:20:43.073182Z [error    ] plan_md_missing_sections       filename=file_validation.py func_name=validate_plan_md_structure lineno=346 missing=['Geometry', 'Objectives']
PASSED
tests/worker_heavy/test_file_validation.py::TestValidatePlanMdStructure::test_valid_engineering_plan 2026-02-22T10:20:43.074458Z [info     ] plan_md_valid                  filename=file_validation.py func_name=validate_plan_md_structure lineno=329 plan_type=engineering
PASSED
tests/worker_heavy/test_file_validation.py::TestValidatePlanMdStructure::test_missing_section_engineering 2026-02-22T10:20:43.075450Z [error    ] plan_md_missing_sections       filename=file_validation.py func_name=validate_plan_md_structure lineno=324 missing=['Missing required section: ## 3. Assembly Strategy', 'Missing required section: ## 4. Cost & Weight Budget', 'Missing required section: ## 5. Risk Assessment']
PASSED
tests/worker_heavy/test_genesis_fem.py::test_genesis_load_scene_soft_mesh 2026-02-22T10:20:43.112202Z [info     ] genesis_using_fem_neohookean   filename=genesis_backend.py func_name=_load_scene_internal lineno=365 mat_id=silicone_rubber name=soft_part
2026-02-22T10:20:43.113382Z [info     ] genesis_building_scene         filename=genesis_backend.py func_name=_load_scene_internal lineno=527
PASSED
tests/worker_heavy/test_genesis_fem.py::test_genesis_step_breakage 2026-02-22T10:20:43.178242Z [warning  ] genesis_step_called_unbuilt_attempting_sync filename=genesis_backend.py func_name=step lineno=543
PASSED
tests/worker_heavy/test_genesis_fem.py::test_get_stress_summaries PASSED
tests/worker_heavy/test_genesis_interaction.py::test_apply_control FAILED
tests/worker_heavy/test_genesis_interaction.py::test_check_collision_with_zone PASSED
tests/worker_heavy/test_handover_objectives.py::test_submit_for_review_includes_objectives None context requested by None
Entering BuildPart with mode=Mode.ADD which is in different scope as parent
WorkplaneList is pushing 1 workplanes: [Plane(o=(0.00, 0.00, 0.00), x=(1.00, 0.00, 0.00), z=(0.00, 0.00, 1.00))]
LocationList is pushing 1 points: [(p=(0.00, 0.00, 0.00), o=(-0.00, 0.00, -0.00))]
BuildPart context requested by Box
Completed integrating 1 object(s) into part with Mode=Mode.ADD
LocationList is popping 1 points
WorkplaneList is popping 1 workplanes
Exiting BuildPart
2026-02-22T10:20:43.422357Z [info     ] handover_started               cwd=. filename=handover.py files=['objectives.yaml', 'assembly_definition.yaml', 'todo.md', 'validation_results.json', 'plan.md', 'renders'] func_name=submit_for_review lineno=24
2026-02-22T10:20:43.424853Z [info     ] objectives_yaml_valid          filename=file_validation.py func_name=validate_objectives_yaml lineno=93
2026-02-22T10:20:43.427850Z [info     ] cost_estimation_yaml_valid     filename=file_validation.py func_name=validate_assembly_definition_yaml lineno=161
2026-02-22T10:20:43.428073Z [info     ] loading_manufacturing_config   filename=config.py func_name=load_config lineno=28 path=/home/maksym/Work/proj/Problemologist/Problemologist-AI/worker_heavy/workbenches/manufacturing_config.yaml
2026-02-22T10:20:43.434242Z [info     ] manufacturing_config_loaded    filename=config.py func_name=load_config lineno=48 methods=['cnc', 'injection_molding', 'three_dp']
2026-02-22T10:20:43.436392Z [info     ] renders_persisted              count=0 filename=handover.py func_name=submit_for_review lineno=160
2026-02-22T10:20:43.437030Z [info     ] handover_complete              filename=handover.py func_name=submit_for_review lineno=186 manifest=/tmp/pytest-of-maksym/pytest-2/test_submit_for_review_include0/renders/review_manifest.json
PASSED
tests/worker_heavy/test_mesh_utils.py::test_repair_mesh_already_watertight PASSED
tests/worker_heavy/test_mesh_utils.py::test_repair_mesh_fixes_inverted_normals PASSED
tests/worker_heavy/test_mesh_utils.py::test_repair_mesh_fails_after_max_attempts Attempt 1/1: Mesh is not watertight or has inconsistent winding, attempting repair
PASSED
tests/worker_heavy/test_mesh_utils.py::test_tetrahedralize_gmsh_success PASSED
tests/worker_heavy/test_mesh_utils.py::test_tetrahedralize_file_not_found PASSED
tests/worker_heavy/test_mesh_utils.py::test_tetrahedralize_invalid_method Tetrahedralization failed using invalid: Unknown tetrahedralization method: invalid
PASSED
tests/worker_heavy/test_model_integration.py::test_model_produced_script_integration None context requested by None
Entering BuildPart with mode=Mode.ADD which is in different scope as parent
WorkplaneList is pushing 1 workplanes: [Plane(o=(0.00, 0.00, 0.00), x=(1.00, 0.00, 0.00), z=(0.00, 0.00, 1.00))]
LocationList is pushing 1 points: [(p=(0.00, 0.00, 0.00), o=(-0.00, 0.00, -0.00))]
BuildPart context requested by None
BuildPart context requested by None
BuildPart context requested by None
Entering BuildSketch with mode=Mode.ADD which is in same scope as parent
WorkplaneList is pushing 1 workplanes: [Plane(o=(0.00, 0.00, 0.00), x=(1.00, 0.00, 0.00), z=(0.00, 0.00, 1.00))]
LocationList is pushing 1 points: [(p=(0.00, 0.00, 0.00), o=(-0.00, 0.00, -0.00))]
BuildSketch context requested by Rectangle
Completed integrating 1 object(s) into part with Mode=Mode.ADD
LocationList is popping 1 points
WorkplaneList is popping 1 workplanes
Exiting BuildSketch
BuildPart context requested by extrude
1 face(s) to extrude on 1 face plane(s)
Completed integrating 1 object(s) into part with Mode=Mode.ADD
LocationList is popping 1 points
WorkplaneList is popping 1 workplanes
Exiting BuildPart
2026-02-22T10:20:43.584672Z [info     ] validate_start                 filename=validation.py func_name=validate lineno=700 particle_budget=None session_id=None smoke_test_mode=True
2026-02-22T10:20:43.585437Z [info     ] prerender_24_views_start       backend=genesis filename=rendering.py func_name=prerender_24_views lineno=45 output_dir=/tmp/pytest-of-maksym/pytest-2/test_model_produced_script_int0/renders particle_budget=None scene_path=None session_id=None smoke_test_mode=True
2026-02-22T10:20:43.589630Z [info     ] loading_manufacturing_config   filename=config.py func_name=load_config lineno=28 path=/home/maksym/Work/proj/Problemologist/Problemologist-AI/worker_heavy/workbenches/manufacturing_config.yaml
2026-02-22T10:20:43.608007Z [info     ] manufacturing_config_loaded    filename=config.py func_name=load_config lineno=48 methods=['cnc', 'injection_molding', 'three_dp']
2026-02-22T10:20:43.628879Z [info     ] loading_manufacturing_config   filename=config.py func_name=load_config lineno=28 path=/home/maksym/Work/proj/Problemologist/Problemologist-AI/worker_heavy/workbenches/manufacturing_config.yaml
2026-02-22T10:20:43.634745Z [info     ] manufacturing_config_loaded    filename=config.py func_name=load_config lineno=48 methods=['cnc', 'injection_molding', 'three_dp']
[Genesis] [10:20:43] [WARNING] Using a simulation timestep smaller than 2ms is not recommended for 'use_gjk_collision=False' as it could lead to numerically unstable collision detection.
Using a simulation timestep smaller than 2ms is not recommended for 'use_gjk_collision=False' as it could lead to numerically unstable collision detection.
[Genesis] [10:20:43] [INFO] Scene <c0c7013> created.
Scene ~~~<<c0c7013>>~~~ created.
[Genesis] [10:20:43] [INFO] Adding <gs.RigidEntity>. idx: 0, uid: <4705607>, morph: <gs.morphs.Plane>, material: <gs.materials.Rigid>.
Adding ~<<gs.RigidEntity>>~. idx: ~<0>~, uid: ~~~<<4705607>>~~~, morph: ~<<gs.morphs.Plane>>~, material: ~<<gs.materials.Rigid>>~.
[Genesis] [10:20:43] [WARNING] Non-zero `coup_restitution` could lead to instability. Use with caution.
Non-zero `coup_restitution` could lead to instability. Use with caution.
[Genesis] [10:20:43] [INFO] Adding <gs.RigidEntity>. idx: 1, uid: <df2a05b>, morph: <gs.morphs.Mesh(file='/tmp/tmp9t2zl2hg/assets/target_box.obj')>, material: <gs.materials.Rigid>.
Adding ~<<gs.RigidEntity>>~. idx: ~<1>~, uid: ~~~<<df2a05b>>~~~, morph: ~<<gs.morphs.Mesh(file='/tmp/tmp9t2zl2hg/assets/target_box.obj')>>~, material: ~<<gs.materials.Rigid>>~.
2026-02-22T10:20:43.684893Z [info     ] genesis_building_scene_for_render_only filename=genesis_backend.py func_name=_load_scene_internal lineno=524
2026-02-22T10:20:43.684997Z [info     ] genesis_building_scene         filename=genesis_backend.py func_name=_load_scene_internal lineno=527
[Genesis] [10:20:43] [INFO] Building scene <c0c7013>...
Building scene ~~~<<c0c7013>>~~~...
[Genesis] [10:20:44] [INFO] Compiling simulation kernels...
Compiling simulation kernels...
[Genesis] [10:20:44] [INFO] Building visualizer...
Building visualizer...
2026-02-22T10:20:44.565818Z [info     ] smoke_test_mode_reducing_render_views filename=rendering.py func_name=prerender_24_views lineno=113
2026-02-22T10:20:44.794663Z [info     ] prerender_complete             count=1 filename=rendering.py func_name=prerender_24_views lineno=165
2026-02-22T10:20:44.795308Z [info     ] simulate_start                 backend=None fem_enabled=None filename=validation.py func_name=simulate lineno=455 particle_budget=None session_id=None smoke_test_mode=True
2026-02-22T10:20:44.795489Z [info     ] DEBUG_simulate                 exists=True filename=validation.py files=[PosixPath('/tmp/pytest-of-maksym/pytest-2/test_model_produced_script_int0/renders')] func_name=simulate lineno=464 working_dir=/tmp/pytest-of-maksym/pytest-2/test_model_produced_script_int0
2026-02-22T10:20:44.796446Z [info     ] loading_manufacturing_config   filename=config.py func_name=load_config lineno=28 path=/home/maksym/Work/proj/Problemologist/Problemologist-AI/worker_heavy/workbenches/manufacturing_config.yaml
2026-02-22T10:20:44.802931Z [info     ] manufacturing_config_loaded    filename=config.py func_name=load_config lineno=48 methods=['cnc', 'injection_molding', 'three_dp']
2026-02-22T10:20:44.815417Z [info     ] loading_manufacturing_config   filename=config.py func_name=load_config lineno=28 path=/home/maksym/Work/proj/Problemologist/Problemologist-AI/worker_heavy/workbenches/manufacturing_config.yaml
2026-02-22T10:20:44.822715Z [info     ] manufacturing_config_loaded    filename=config.py func_name=load_config lineno=48 methods=['cnc', 'injection_molding', 'three_dp']
[Genesis] [10:20:44] [WARNING] Using a simulation timestep smaller than 2ms is not recommended for 'use_gjk_collision=False' as it could lead to numerically unstable collision detection.
Using a simulation timestep smaller than 2ms is not recommended for 'use_gjk_collision=False' as it could lead to numerically unstable collision detection.
[Genesis] [10:20:44] [INFO] Scene <c02f132> created.
Scene ~~~<<c02f132>>~~~ created.
[Genesis] [10:20:44] [INFO] Adding <gs.RigidEntity>. idx: 0, uid: <d1fbc87>, morph: <gs.morphs.Plane>, material: <gs.materials.Rigid>.
Adding ~<<gs.RigidEntity>>~. idx: ~<0>~, uid: ~~~<<d1fbc87>>~~~, morph: ~<<gs.morphs.Plane>>~, material: ~<<gs.materials.Rigid>>~.
[Genesis] [10:20:44] [WARNING] Non-zero `coup_restitution` could lead to instability. Use with caution.
Non-zero `coup_restitution` could lead to instability. Use with caution.
[Genesis] [10:20:44] [INFO] Adding <gs.RigidEntity>. idx: 1, uid: <fa45a67>, morph: <gs.morphs.Mesh(file='/tmp/pytest-of-maksym/pytest-2/test_model_produced_script_int0/assets/target_box.obj')>, material: <gs.materials.Rigid>.
Adding ~<<gs.RigidEntity>>~. idx: ~<1>~, uid: ~~~<<fa45a67>>~~~, morph: ~<<gs.morphs.Mesh(file='/tmp/pytest-of-maksym/pytest-2/test_model_produced_script_int0/assets/target_box.obj')>>~, material: ~<<gs.materials.Rigid>>~.
2026-02-22T10:20:44.857328Z [info     ] genesis_building_scene         filename=genesis_backend.py func_name=_load_scene_internal lineno=527
[Genesis] [10:20:44] [INFO] Building scene <c02f132>...
Building scene ~~~<<c02f132>>~~~...
[Genesis] [10:20:45] [INFO] Compiling simulation kernels...
Compiling simulation kernels...
[Genesis] [10:20:45] [INFO] Building visualizer...
Building visualizer...
2026-02-22T10:20:45.700328Z [info     ] loading_manufacturing_config   filename=config.py func_name=load_config lineno=28 path=/home/maksym/Work/proj/Problemologist/Problemologist-AI/worker_heavy/workbenches/manufacturing_config.yaml
2026-02-22T10:20:45.706214Z [info     ] manufacturing_config_loaded    filename=config.py func_name=load_config lineno=48 methods=['cnc', 'injection_molding', 'three_dp']
2026-02-22T10:20:45.706331Z [info     ] loop_loaded_default_config     filename=loop.py func_name=__init__ lineno=133 materials=['aluminum_6061', 'abs', 'silicone_rubber']
2026-02-22T10:20:45.706407Z [info     ] starting_dfm_facade_analysis   fem_required=False filename=dfm.py func_name=validate_and_price lineno=116 method=3dp
2026-02-22T10:20:45.706471Z [info     ] starting_3dp_analysis          filename=print_3d.py func_name=analyze_3dp lineno=125
2026-02-22T10:20:45.708580Z [info     ] calculating_3dp_cost           filename=print_3d.py func_name=calculate_3dp_cost lineno=62 material=abs quantity=1
2026-02-22T10:20:45.709523Z [info     ] 3dp_analysis_complete          filename=print_3d.py func_name=analyze_3dp is_manufacturable=True lineno=175 violations=0
2026-02-22T10:20:45.709690Z [info     ] SimulationLoop_init            filename=loop.py forbidden_sites=[] func_name=__init__ goal_sites=[] lineno=188
2026-02-22T10:20:45.709864Z [info     ] SimulationLoop_step_start      filename=loop.py func_name=step lineno=354 target_body_name=target_box
[Genesis] [10:20:45] [INFO] Running at 152.64 FPS (152.64 FPS per env, 1 envs).
Running at ~<152.64>~ FPS (~<152.64>~ FPS per env, ~<1>~ envs).
[Genesis] [10:20:45] [INFO] Running at 171.56 FPS (171.56 FPS per env, 1 envs).
Running at ~<171.56>~ FPS (~<171.56>~ FPS per env, ~<1>~ envs).
[Genesis] [10:20:45] [INFO] Running at 171.62 FPS (171.62 FPS per env, 1 envs).
Running at ~<171.62>~ FPS (~<171.62>~ FPS per env, ~<1>~ envs).
[Genesis] [10:20:45] [INFO] Running at 171.56 FPS (171.56 FPS per env, 1 envs).
Running at ~<171.56>~ FPS (~<171.56>~ FPS per env, ~<1>~ envs).
[Genesis] [10:20:45] [INFO] Running at 171.01 FPS (171.01 FPS per env, 1 envs).
Running at ~<171.01>~ FPS (~<171.01>~ FPS per env, ~<1>~ envs).
[Genesis] [10:20:46] [INFO] Running at 151.93 FPS (151.93 FPS per env, 1 envs).
Running at ~<151.93>~ FPS (~<151.93>~ FPS per env, ~<1>~ envs).
[Genesis] [10:20:46] [INFO] Running at 170.76 FPS (170.76 FPS per env, 1 envs).
Running at ~<170.76>~ FPS (~<170.76>~ FPS per env, ~<1>~ envs).
[Genesis] [10:20:46] [INFO] Running at 170.75 FPS (170.75 FPS per env, 1 envs).
Running at ~<170.75>~ FPS (~<170.75>~ FPS per env, ~<1>~ envs).
[Genesis] [10:20:46] [INFO] Running at 170.75 FPS (170.75 FPS per env, 1 envs).
Running at ~<170.75>~ FPS (~<170.75>~ FPS per env, ~<1>~ envs).
[Genesis] [10:20:46] [INFO] Running at 170.79 FPS (170.79 FPS per env, 1 envs).
Running at ~<170.79>~ FPS (~<170.79>~ FPS per env, ~<1>~ envs).
[Genesis] [10:20:46] [INFO] Running at 170.73 FPS (170.73 FPS per env, 1 envs).
Running at ~<170.73>~ FPS (~<170.73>~ FPS per env, ~<1>~ envs).
[Genesis] [10:20:46] [INFO] Running at 170.59 FPS (170.59 FPS per env, 1 envs).
Running at ~<170.59>~ FPS (~<170.59>~ FPS per env, ~<1>~ envs).
[Genesis] [10:20:46] [INFO] Running at 170.41 FPS (170.41 FPS per env, 1 envs).
Running at ~<170.41>~ FPS (~<170.41>~ FPS per env, ~<1>~ envs).
[Genesis] [10:20:46] [INFO] Running at 170.22 FPS (170.22 FPS per env, 1 envs).
Running at ~<170.22>~ FPS (~<170.22>~ FPS per env, ~<1>~ envs).
[Genesis] [10:20:46] [INFO] Running at 169.65 FPS (169.65 FPS per env, 1 envs).
Running at ~<169.65>~ FPS (~<169.65>~ FPS per env, ~<1>~ envs).
[Genesis] [10:20:46] [INFO] Running at 169.44 FPS (169.44 FPS per env, 1 envs).
Running at ~<169.44>~ FPS (~<169.44>~ FPS per env, ~<1>~ envs).
[Genesis] [10:20:46] [INFO] Running at 169.29 FPS (169.29 FPS per env, 1 envs).
Running at ~<169.29>~ FPS (~<169.29>~ FPS per env, ~<1>~ envs).
[Genesis] [10:20:46] [INFO] Running at 169.21 FPS (169.21 FPS per env, 1 envs).
Running at ~<169.21>~ FPS (~<169.21>~ FPS per env, ~<1>~ envs).
[Genesis] [10:20:46] [INFO] Running at 168.69 FPS (168.69 FPS per env, 1 envs).
Running at ~<168.69>~ FPS (~<168.69>~ FPS per env, ~<1>~ envs).
[Genesis] [10:20:46] [INFO] Running at 150.16 FPS (150.16 FPS per env, 1 envs).
Running at ~<150.16>~ FPS (~<150.16>~ FPS per env, ~<1>~ envs).
[Genesis] [10:20:46] [INFO] Running at 168.66 FPS (168.66 FPS per env, 1 envs).
Running at ~<168.66>~ FPS (~<168.66>~ FPS per env, ~<1>~ envs).
[Genesis] [10:20:46] [INFO] Running at 168.55 FPS (168.55 FPS per env, 1 envs).
Running at ~<168.55>~ FPS (~<168.55>~ FPS per env, ~<1>~ envs).
[Genesis] [10:20:46] [INFO] Running at 168.43 FPS (168.43 FPS per env, 1 envs).
Running at ~<168.43>~ FPS (~<168.43>~ FPS per env, ~<1>~ envs).
[Genesis] [10:20:47] [INFO] Running at 168.39 FPS (168.39 FPS per env, 1 envs).
Running at ~<168.39>~ FPS (~<168.39>~ FPS per env, ~<1>~ envs).
[Genesis] [10:20:47] [INFO] Running at 168.28 FPS (168.28 FPS per env, 1 envs).
Running at ~<168.28>~ FPS (~<168.28>~ FPS per env, ~<1>~ envs).
[Genesis] [10:20:47] [INFO] Running at 168.23 FPS (168.23 FPS per env, 1 envs).
Running at ~<168.23>~ FPS (~<168.23>~ FPS per env, ~<1>~ envs).
[Genesis] [10:20:47] [INFO] Running at 168.10 FPS (168.10 FPS per env, 1 envs).
Running at ~<168.10>~ FPS (~<168.10>~ FPS per env, ~<1>~ envs).
[Genesis] [10:20:47] [INFO] Running at 168.02 FPS (168.02 FPS per env, 1 envs).
Running at ~<168.02>~ FPS (~<168.02>~ FPS per env, ~<1>~ envs).
2026-02-22T10:20:47.223285Z [info     ] loading_manufacturing_config   filename=config.py func_name=load_config lineno=28 path=/home/maksym/Work/proj/Problemologist/Problemologist-AI/worker_heavy/workbenches/manufacturing_config.yaml
2026-02-22T10:20:47.228733Z [info     ] manufacturing_config_loaded    filename=config.py func_name=load_config lineno=48 methods=['cnc', 'injection_molding', 'three_dp']
2026-02-22T10:20:47.228848Z [info     ] starting_dfm_facade_analysis   fem_required=False filename=dfm.py func_name=validate_and_price lineno=116 method=3dp
2026-02-22T10:20:47.228916Z [info     ] starting_3dp_analysis          filename=print_3d.py func_name=analyze_3dp lineno=125
2026-02-22T10:20:47.230995Z [info     ] calculating_3dp_cost           filename=print_3d.py func_name=calculate_3dp_cost lineno=62 material=abs quantity=1
2026-02-22T10:20:47.231912Z [info     ] 3dp_analysis_complete          filename=print_3d.py func_name=analyze_3dp is_manufacturable=True lineno=175 violations=0
PASSED
tests/worker_heavy/test_node_validation.py::test_validate_node_output_planner_success 2026-02-22T10:20:47.279673Z [info     ] plan_md_valid                  filename=file_validation.py func_name=validate_plan_md_structure lineno=329 plan_type=engineering
2026-02-22T10:20:47.280500Z [info     ] cost_estimation_yaml_valid     filename=file_validation.py func_name=validate_assembly_definition_yaml lineno=161
PASSED
tests/worker_heavy/test_node_validation.py::test_validate_node_output_planner_missing_file 2026-02-22T10:20:47.281702Z [error    ] plan_md_missing_sections       filename=file_validation.py func_name=validate_plan_md_structure lineno=324 missing=['Missing required section: ## 2. Parts List', 'Missing required section: ## 3. Assembly Strategy', 'Missing required section: ## 4. Cost & Weight Budget', 'Missing required section: ## 5. Risk Assessment']
PASSED
tests/worker_heavy/test_node_validation.py::test_validate_node_output_template_placeholder 2026-02-22T10:20:47.283062Z [error    ] plan_md_missing_sections       filename=file_validation.py func_name=validate_plan_md_structure lineno=324 missing=['Missing required section: ## 2. Parts List', 'Missing required section: ## 3. Assembly Strategy', 'Missing required section: ## 4. Cost & Weight Budget', 'Missing required section: ## 5. Risk Assessment']
PASSED
tests/worker_heavy/test_node_validation.py::test_validate_node_output_invalid_plan_structure 2026-02-22T10:20:47.284361Z [error    ] plan_md_missing_sections       filename=file_validation.py func_name=validate_plan_md_structure lineno=324 missing=['Missing required section: ## 1. Solution Overview', 'Missing required section: ## 2. Parts List', 'Missing required section: ## 3. Assembly Strategy', 'Missing required section: ## 4. Cost & Weight Budget', 'Missing required section: ## 5. Risk Assessment']
PASSED
tests/worker_heavy/test_node_validation.py::test_validate_node_output_coder_success 2026-02-22T10:20:47.285730Z [info     ] plan_md_valid                  filename=file_validation.py func_name=validate_plan_md_structure lineno=329 plan_type=engineering
2026-02-22T10:20:47.288903Z [info     ] objectives_yaml_valid          filename=file_validation.py func_name=validate_objectives_yaml lineno=93
PASSED
tests/worker_heavy/test_node_validation.py::test_validate_node_output_objectives_template 2026-02-22T10:20:47.289983Z [error    ] plan_md_missing_sections       filename=file_validation.py func_name=validate_plan_md_structure lineno=324 missing=['Missing required section: ## 2. Parts List', 'Missing required section: ## 3. Assembly Strategy', 'Missing required section: ## 4. Cost & Weight Budget', 'Missing required section: ## 5. Risk Assessment']
PASSED
tests/worker_heavy/test_physics_backends.py::test_simulation_loop_with_mujoco 2026-02-22T10:20:47.325356Z [info     ] SimulationLoop_init            filename=loop.py forbidden_sites=[] func_name=__init__ goal_sites=[] lineno=188
2026-02-22T10:20:47.325548Z [info     ] SimulationLoop_step_start      filename=loop.py func_name=step lineno=354 target_body_name=None
PASSED
tests/worker_heavy/test_physics_backends.py::test_simulation_builder_factory PASSED
tests/worker_heavy/test_physics_backends.py::test_factory_mujoco PASSED
tests/worker_heavy/test_physics_backends.py::test_factory_genesis PASSED
tests/worker_heavy/test_rendering_snapshot.py::test_render_selection_snapshot 2026-02-22T10:20:47.562750Z [info     ] rendering_selection_snapshot   filename=rendering.py func_name=render_selection_snapshot ids=['face_0'] lineno=34
None context requested by None
Entering BuildPart with mode=Mode.ADD which is in different scope as parent
WorkplaneList is pushing 1 workplanes: [Plane(o=(0.00, 0.00, 0.00), x=(1.00, 0.00, 0.00), z=(0.00, 0.00, 1.00))]
LocationList is pushing 1 points: [(p=(0.00, 0.00, 0.00), o=(-0.00, 0.00, -0.00))]
BuildPart context requested by Box
Completed integrating 1 object(s) into part with Mode=Mode.ADD
LocationList is popping 1 points
WorkplaneList is popping 1 workplanes
Exiting BuildPart
2026-02-22 10:20:47.682 ( 177.433s) [    71A741195080]vtkOpenGLRenderWindow.c:701    ERR| vtkXOpenGLRenderWindow (0x4dba18f0): GLEW could not be initialized: Unknown error
2026-02-22 10:20:47.682 ( 177.433s) [    71A741195080]     vtkOpenGLState.cxx:1787  WARN| Hardware does not support the number of textures defined.
2026-02-22 10:20:47.682 ( 177.433s) [    71A741195080]     vtkOpenGLState.cxx:1787  WARN| Hardware does not support the number of textures defined.
2026-02-22 10:20:47.689 ( 177.440s) [    71A741195080]     vtkOpenGLState.cxx:1787  WARN| Hardware does not support the number of textures defined.
2026-02-22 10:20:47.689 ( 177.440s) [    71A741195080]     vtkOpenGLState.cxx:1787  WARN| Hardware does not support the number of textures defined.
2026-02-22 10:20:47.689 ( 177.440s) [    71A741195080]     vtkOpenGLState.cxx:1787  WARN| Hardware does not support the number of textures defined.
2026-02-22 10:20:47.690 ( 177.441s) [    71A741195080]     vtkOpenGLState.cxx:1787  WARN| Hardware does not support the number of textures defined.
2026-02-22 10:20:47.690 ( 177.441s) [    71A741195080]     vtkOpenGLState.cxx:1787  WARN| Hardware does not support the number of textures defined.
2026-02-22 10:20:47.691 ( 177.442s) [    71A741195080]     vtkOpenGLState.cxx:1787  WARN| Hardware does not support the number of textures defined.
2026-02-22 10:20:47.702 ( 177.452s) [    71A741195080]     vtkOpenGLState.cxx:1787  WARN| Hardware does not support the number of textures defined.
2026-02-22 10:20:47.702 ( 177.453s) [    71A741195080]     vtkOpenGLState.cxx:1787  WARN| Hardware does not support the number of textures defined.
2026-02-22 10:20:47.702 ( 177.453s) [    71A741195080]     vtkOpenGLState.cxx:1787  WARN| Hardware does not support the number of textures defined.
2026-02-22 10:20:47.706 ( 177.457s) [    71A741195080]     vtkOpenGLState.cxx:1787  WARN| Hardware does not support the number of textures defined.
2026-02-22 10:20:47.706 ( 177.457s) [    71A741195080]     vtkOpenGLState.cxx:1787  WARN| Hardware does not support the number of textures defined.
2026-02-22 10:20:47.710 ( 177.461s) [    71A741195080]     vtkOpenGLState.cxx:1787  WARN| Hardware does not support the number of textures defined.
2026-02-22 10:20:47.713 ( 177.464s) [    71A741195080]     vtkOpenGLState.cxx:1787  WARN| Hardware does not support the number of textures defined.
2026-02-22 10:20:47.713 ( 177.464s) [    71A741195080]     vtkOpenGLState.cxx:1787  WARN| Hardware does not support the number of textures defined.
2026-02-22 10:20:47.713 ( 177.464s) [    71A741195080]     vtkOpenGLState.cxx:1787  WARN| Hardware does not support the number of textures defined.
2026-02-22T10:20:47.743970Z [info     ] snapshot_uploaded              filename=rendering.py func_name=render_selection_snapshot key=snapshots/5c935da5-e4fd-4232-90f4-b12c645603c5.png lineno=145
PASSED
tests/worker_heavy/test_simulation_builder_electronics.py::test_builder_wire_tendons None context requested by Box
None context requested by Box
Adding free joint to body 'comp1'. This part will fall if not supported. Use 'fixed=True' or 'constraint' attribute to secure it.
Adding free joint to body 'comp2'. This part will fall if not supported. Use 'fixed=True' or 'constraint' attribute to secure it.
PASSED
tests/worker_heavy/test_simulation_persistence.py::test_simulation_persistence None context requested by Box
2026-02-22T10:20:47.882240Z [info     ] simulate_start                 backend=None fem_enabled=None filename=validation.py func_name=simulate lineno=455 particle_budget=None session_id=None smoke_test_mode=True
2026-02-22T10:20:47.882438Z [info     ] DEBUG_simulate                 exists=True filename=validation.py files=[] func_name=simulate lineno=464 working_dir=/tmp/pytest-of-maksym/pytest-2/test_simulation_persistence0
2026-02-22T10:20:47.886326Z [info     ] rendering_stress_heatmaps      count=1 filename=validation.py func_name=preview_stress lineno=124
2026-02-22T10:20:48.041537Z [info     ] rendering_stress_heatmaps      count=1 filename=validation.py func_name=preview_stress lineno=124
PASSED
tests/worker_heavy/test_topology.py::test_inspect_topology_face None context requested by None
Entering BuildPart with mode=Mode.ADD which is in different scope as parent
WorkplaneList is pushing 1 workplanes: [Plane(o=(0.00, 0.00, 0.00), x=(1.00, 0.00, 0.00), z=(0.00, 0.00, 1.00))]
LocationList is pushing 1 points: [(p=(0.00, 0.00, 0.00), o=(-0.00, 0.00, -0.00))]
BuildPart context requested by Box
Completed integrating 1 object(s) into part with Mode=Mode.ADD
LocationList is popping 1 points
WorkplaneList is popping 1 workplanes
Exiting BuildPart
PASSED
tests/worker_heavy/test_topology.py::test_inspect_topology_part None context requested by None
Entering BuildPart with mode=Mode.ADD which is in different scope as parent
WorkplaneList is pushing 1 workplanes: [Plane(o=(0.00, 0.00, 0.00), x=(1.00, 0.00, 0.00), z=(0.00, 0.00, 1.00))]
LocationList is pushing 1 points: [(p=(0.00, 0.00, 0.00), o=(-0.00, 0.00, -0.00))]
BuildPart context requested by Box
Completed integrating 1 object(s) into part with Mode=Mode.ADD
LocationList is popping 1 points
WorkplaneList is popping 1 workplanes
Exiting BuildPart
PASSED
tests/worker_heavy/test_topology.py::test_inspect_topology_invalid None context requested by Box
PASSED
tests/worker_heavy/test_validation.py::test_geometric_validation None context requested by None
Entering BuildPart with mode=Mode.ADD which is in different scope as parent
WorkplaneList is pushing 1 workplanes: [Plane(o=(0.00, 0.00, 0.00), x=(1.00, 0.00, 0.00), z=(0.00, 0.00, 1.00))]
LocationList is pushing 1 points: [(p=(0.00, 0.00, 0.00), o=(-0.00, 0.00, -0.00))]
BuildPart context requested by Box
Completed integrating 1 object(s) into part with Mode=Mode.ADD
LocationList is popping 1 points
WorkplaneList is popping 1 workplanes
Exiting BuildPart
2026-02-22T10:20:48.224895Z [info     ] validate_start                 filename=validation.py func_name=validate lineno=700 particle_budget=None session_id=None smoke_test_mode=True
2026-02-22T10:20:48.225128Z [info     ] prerender_24_views_start       backend=genesis filename=rendering.py func_name=prerender_24_views lineno=45 output_dir=/tmp/pytest-of-maksym/pytest-2/test_simulation_persistence0/renders particle_budget=None scene_path=None session_id=None smoke_test_mode=True
2026-02-22T10:20:48.226136Z [error    ] prerender_failed               error="Part 'unknown' is missing required metadata. Every part must have a .metadata attribute (PartMetadata or CompoundMetadata)." filename=rendering.py func_name=prerender_24_views lineno=170
Traceback (most recent call last):
  File "/home/maksym/Work/proj/Problemologist/Problemologist-AI/worker_heavy/utils/rendering.py", line 68, in prerender_24_views
    final_scene_path = builder.build_from_assembly(
                       ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "/home/maksym/Work/proj/Problemologist/Problemologist-AI/worker_heavy/simulation/builder.py", line 920, in build_from_assembly
    parts_data = CommonAssemblyTraverser.traverse(assembly, electronics)
                 ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "/home/maksym/Work/proj/Problemologist/Problemologist-AI/worker_heavy/simulation/builder.py", line 67, in traverse
    meta = CommonAssemblyTraverser._resolve_part_metadata(child)
           ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "/home/maksym/Work/proj/Problemologist/Problemologist-AI/worker_heavy/simulation/builder.py", line 132, in _resolve_part_metadata
    raise ValueError(
ValueError: Part 'unknown' is missing required metadata. Every part must have a .metadata attribute (PartMetadata or CompoundMetadata).

2026-02-22T10:20:48.226247Z [warning  ] validate_render_capture_failed error="Part 'unknown' is missing required metadata. Every part must have a .metadata attribute (PartMetadata or CompoundMetadata)." filename=validation.py func_name=validate lineno=852
None context requested by None
Entering BuildPart with mode=Mode.ADD which is in different scope as parent
WorkplaneList is pushing 1 workplanes: [Plane(o=(0.00, 0.00, 0.00), x=(1.00, 0.00, 0.00), z=(0.00, 0.00, 1.00))]
LocationList is pushing 1 points: [(p=(0.00, 0.00, 0.00), o=(-0.00, 0.00, -0.00))]
BuildPart context requested by Box
Completed integrating 1 object(s) into part with Mode=Mode.ADD
LocationList is popping 1 points
WorkplaneList is popping 1 workplanes
Exiting BuildPart
None context requested by None
Entering BuildPart with mode=Mode.ADD which is in different scope as parent
WorkplaneList is pushing 1 workplanes: [Plane(o=(0.00, 0.00, 0.00), x=(1.00, 0.00, 0.00), z=(0.00, 0.00, 1.00))]
LocationList is pushing 1 points: [(p=(0.00, 0.00, 0.00), o=(-0.00, 0.00, -0.00))]
BuildPart context requested by Box
Completed integrating 1 object(s) into part with Mode=Mode.ADD
LocationList is popping 1 points
WorkplaneList is popping 1 workplanes
Exiting BuildPart
2026-02-22T10:20:48.235438Z [info     ] validate_start                 filename=validation.py func_name=validate lineno=700 particle_budget=None session_id=None smoke_test_mode=True
PASSED
tests/worker_heavy/test_validation.py::test_objectives_validation_events 2026-02-22T10:20:48.248265Z [error    ] objectives_yaml_parse_error    error='while parsing a flow node\nexpected the node content, but found \'<stream end>\'\n  in "<unicode string>", line 1, column 13:\n    invalid: { [\n                ^' filename=file_validation.py func_name=validate_objectives_yaml lineno=96
2026-02-22T10:20:48.248716Z [error    ] objectives_yaml_validation_error errors=["('objectives',): Field required", "('simulation_bounds',): Field required", "('moved_object',): Field required", "('constraints',): Field required"] filename=file_validation.py func_name=validate_objectives_yaml lineno=100
PASSED
tests/worker_heavy/test_validation.py::test_plan_validation_events 2026-02-22T10:20:48.250716Z [error    ] plan_md_missing_sections       filename=file_validation.py func_name=validate_plan_md_structure lineno=346 missing=['Learning Objective', 'Geometry', 'Objectives']
PASSED
tests/worker_heavy/test_validation.py::test_simulation None context requested by None
Entering BuildPart with mode=Mode.ADD which is in different scope as parent
WorkplaneList is pushing 1 workplanes: [Plane(o=(0.00, 0.00, 0.00), x=(1.00, 0.00, 0.00), z=(0.00, 0.00, 1.00))]
LocationList is pushing 1 points: [(p=(0.00, 0.00, 0.00), o=(-0.00, 0.00, -0.00))]
BuildPart context requested by Box
Completed integrating 1 object(s) into part with Mode=Mode.ADD
LocationList is popping 1 points
WorkplaneList is popping 1 workplanes
Exiting BuildPart
2026-02-22T10:20:48.295453Z [info     ] simulate_start                 backend=None fem_enabled=None filename=validation.py func_name=simulate lineno=455 particle_budget=None session_id=None smoke_test_mode=True
2026-02-22T10:20:48.295654Z [info     ] DEBUG_simulate                 exists=True filename=validation.py files=[PosixPath('/tmp/pytest-of-maksym/pytest-2/test_simulation_persistence0/simulation_result.json'), PosixPath('/tmp/pytest-of-maksym/pytest-2/test_simulation_persistence0/renders')] func_name=simulate lineno=464 working_dir=/tmp/pytest-of-maksym/pytest-2/test_simulation_persistence0
FAILED
tests/worker_heavy/test_validation.py::test_handover None context requested by None
Entering BuildPart with mode=Mode.ADD which is in different scope as parent
WorkplaneList is pushing 1 workplanes: [Plane(o=(0.00, 0.00, 0.00), x=(1.00, 0.00, 0.00), z=(0.00, 0.00, 1.00))]
LocationList is pushing 1 points: [(p=(0.00, 0.00, 0.00), o=(-0.00, 0.00, -0.00))]
BuildPart context requested by Box
Completed integrating 1 object(s) into part with Mode=Mode.ADD
LocationList is popping 1 points
WorkplaneList is popping 1 workplanes
Exiting BuildPart
2026-02-22T10:20:48.374660Z [info     ] handover_started               cwd=. filename=handover.py files=['roadmap.md', 'worker_openapi.json', '.env.example', 'assets', 'controller_openapi.json', '.pre-commit-config.yaml', 'non_existent.db', '.codex', '__pycache__', 'history.db', 'events.jsonl', 'objectives.yaml', 'docs', 'README.md', 'code-reviews', 'assembly_definition.yaml', 'test_output.log', 'temp_baseline.obj', '.hypothesis', 'skills', 'shared', '.agent', 'test_dir', 'worker_heavy', 'MUJOCO_LOG.TXT', '.venv', 'todo.md', '.gitattributes', 'project-ideas.md', 'pyproject.toml', '.kittify', '.claude', 'test_output', 'frontend', 'controller', 'docker-compose.test.yaml', '.git', 'pyrightconfig.json', 'file-description.md', '.gemini', 'evals', 'scripts', 'specs', '.vscode', 'docker-compose.yml', 'Podmanfile.sandbox', 'test_suggested_skills', 'kitty-specs', 'ruff.toml', 'tests', 'GEMINI.md', 'worker_light', 'agent_runtime_utils', 'main.py', '.env', 'uv.lock', 'validation_results.json', '.pytest_cache', 'logs', 'plan.md', '.github', '.claudeignore', '.gitignore', 'alembic.ini', 'worker', 'output.log', 'config', '.zed', '.jules', 'suggested_skills', '.ruff_cache', 'renders', '.dockerignore', 'parts.db', '.python-version'] func_name=submit_for_review lineno=24
2026-02-22T10:20:48.377797Z [info     ] objectives_yaml_valid          filename=file_validation.py func_name=validate_objectives_yaml lineno=93
2026-02-22T10:20:48.416778Z [info     ] cost_estimation_yaml_valid     filename=file_validation.py func_name=validate_assembly_definition_yaml lineno=161
2026-02-22T10:20:48.416964Z [info     ] loading_manufacturing_config   filename=config.py func_name=load_config lineno=28 path=/home/maksym/Work/proj/Problemologist/Problemologist-AI/worker_heavy/workbenches/manufacturing_config.yaml
2026-02-22T10:20:48.423614Z [info     ] manufacturing_config_loaded    filename=config.py func_name=load_config lineno=48 methods=['cnc', 'injection_molding', 'three_dp']
2026-02-22T10:20:48.425709Z [info     ] renders_persisted              count=0 filename=handover.py func_name=submit_for_review lineno=160
2026-02-22T10:20:48.469533Z [info     ] handover_complete              filename=handover.py func_name=submit_for_review lineno=186 manifest=/tmp/pytest-of-maksym/pytest-2/test_simulation_persistence0/renders/review_manifest.json
PASSED
tests/worker_heavy/test_wire_clearance.py::test_wire_clearance_violation None context requested by Box
2026-02-22T10:20:48.477106Z [info     ] loop_loaded_default_config     filename=loop.py func_name=__init__ lineno=133 materials=[]
2026-02-22T10:20:48.477324Z [info     ] SimulationLoop_init            filename=loop.py forbidden_sites=[] func_name=__init__ goal_sites=[] lineno=188
None context requested by None
None context requested by Polyline
Wire clearance violation: 0.26mm at Vector(-4.7368421052632, 0, 0)
FAILED
tests/worker_heavy/test_wire_clearance.py::test_wire_clearance_success None context requested by Box
2026-02-22T10:20:48.496028Z [info     ] loop_loaded_default_config     filename=loop.py func_name=__init__ lineno=133 materials=[]
2026-02-22T10:20:48.496205Z [info     ] SimulationLoop_init            filename=loop.py forbidden_sites=[] func_name=__init__ goal_sites=[] lineno=188
None context requested by None
None context requested by Polyline
PASSED
tests/worker_heavy/test_worker_utils.py::test_submit_for_review 2026-02-22T10:20:48.510344Z [info     ] handover_started               cwd=/tmp/pytest-of-maksym/pytest-2/test_submit_for_review0 filename=handover.py files=['objectives.yaml', 'assembly_definition.yaml', 'todo.md', 'validation_results.json', 'plan.md', 'renders'] func_name=submit_for_review lineno=24
2026-02-22T10:20:48.516979Z [info     ] objectives_yaml_valid          filename=file_validation.py func_name=validate_objectives_yaml lineno=93
2026-02-22T10:20:48.519771Z [info     ] cost_estimation_yaml_valid     filename=file_validation.py func_name=validate_assembly_definition_yaml lineno=161
2026-02-22T10:20:48.519955Z [info     ] loading_manufacturing_config   filename=config.py func_name=load_config lineno=28 path=/home/maksym/Work/proj/Problemologist/Problemologist-AI/worker_heavy/workbenches/manufacturing_config.yaml
2026-02-22T10:20:48.525646Z [info     ] manufacturing_config_loaded    filename=config.py func_name=load_config lineno=48 methods=['cnc', 'injection_molding', 'three_dp']
2026-02-22T10:20:48.528141Z [info     ] renders_persisted              count=0 filename=handover.py func_name=submit_for_review lineno=160
2026-02-22T10:20:48.528884Z [info     ] handover_complete              filename=handover.py func_name=submit_for_review lineno=186 manifest=/tmp/pytest-of-maksym/pytest-2/test_submit_for_review0/renders/review_manifest.json
PASSED
tests/worker_heavy/test_worker_utils.py::test_prerender_24_views None context requested by None
Entering BuildPart with mode=Mode.ADD which is in different scope as parent
WorkplaneList is pushing 1 workplanes: [Plane(o=(0.00, 0.00, 0.00), x=(1.00, 0.00, 0.00), z=(0.00, 0.00, 1.00))]
LocationList is pushing 1 points: [(p=(0.00, 0.00, 0.00), o=(-0.00, 0.00, -0.00))]
BuildPart context requested by Box
Completed integrating 1 object(s) into part with Mode=Mode.ADD
LocationList is popping 1 points
WorkplaneList is popping 1 workplanes
Exiting BuildPart
2026-02-22T10:20:48.535604Z [info     ] prerender_24_views_start       backend=genesis filename=rendering.py func_name=prerender_24_views lineno=45 output_dir=/tmp/pytest-of-maksym/pytest-2/test_prerender_24_views0 particle_budget=None scene_path=None session_id=None smoke_test_mode=True
2026-02-22T10:20:48.536305Z [info     ] smoke_test_mode_reducing_render_views filename=rendering.py func_name=prerender_24_views lineno=113
2026-02-22T10:20:48.542750Z [info     ] prerender_complete             count=1 filename=rendering.py func_name=prerender_24_views lineno=165
FAILED
tests/worker_heavy/utils/test_controllers.py::test_constant_controller PASSED
tests/worker_heavy/utils/test_controllers.py::test_sinusoidal_controller PASSED
tests/worker_heavy/utils/test_controllers.py::test_square_controller PASSED
tests/worker_heavy/utils/test_controllers.py::test_trapezoidal_controller PASSED
tests/worker_heavy/utils/test_controllers.py::test_rotate_to_controller PASSED
tests/worker_heavy/utils/test_physics_utils.py::test_get_stress_report_logic PASSED
tests/worker_heavy/utils/test_physics_utils.py::test_define_fluid PASSED
tests/worker_heavy/utils/test_physics_utils.py::test_set_soft_mesh 2026-02-22T10:20:48.569392Z [info     ] set_soft_mesh_enabled          fem_enabled=True filename=validation.py func_name=set_soft_mesh lineno=218 part_id=part1
PASSED
tests/worker_light/test_api.py::test_health PASSED
tests/worker_light/test_api.py::test_fs_ls PASSED
tests/worker_light/test_api.py::test_fs_read PASSED
tests/worker_light/test_api.py::test_fs_write PASSED
tests/worker_light/test_api.py::test_fs_exists PASSED
tests/worker_light/test_api.py::test_fs_edit PASSED
tests/worker_light/test_api.py::test_execute_runtime 2026-02-22T10:20:48.623151Z [info     ] runtime_execute_async_complete exit_code=0 filename=executor.py func_name=run_python_code_async lineno=217 session_id=test-session stderr_length=0 stdout_length=15
PASSED
tests/worker_light/test_api.py::test_execute_runtime_timeout 2026-02-22T10:20:49.631996Z [warning  ] runtime_execute_async_timeout  filename=executor.py func_name=run_python_code_async lineno=241 session_id=test-session timeout=1
PASSED
tests/worker_light/test_binary_transfer.py::test_router_upload_files_binary 2026-02-22T10:20:49.640385Z [warning  ] router_upload_blocked          filename=router.py func_name=upload_files lineno=275 path=/readonly/test.bin
PASSED
tests/worker_light/test_binary_transfer.py::test_worker_client_upload_file FAILED
tests/worker_light/test_binary_transfer.py::test_worker_client_read_file_binary FAILED
tests/worker_light/test_core.py::TestRuntimeExecutor::test_run_simple_code 2026-02-22T10:20:49.704495Z [info     ] runtime_execute_complete       exit_code=0 filename=executor.py func_name=run_python_code lineno=107 stderr_length=0 stdout_length=6
PASSED
tests/worker_light/test_core.py::TestRuntimeExecutor::test_run_code_with_error 2026-02-22T10:20:49.721755Z [info     ] runtime_execute_complete       exit_code=1 filename=executor.py func_name=run_python_code lineno=107 stderr_length=143 stdout_length=0
PASSED
tests/worker_light/test_core.py::TestRuntimeExecutor::test_run_code_timeout 2026-02-22T10:20:50.726392Z [warning  ] runtime_execute_timeout        filename=executor.py func_name=run_python_code lineno=122 timeout=1
PASSED
tests/worker_light/test_core.py::TestRuntimeExecutor::test_run_code_with_env 2026-02-22T10:20:50.749219Z [info     ] runtime_execute_complete       exit_code=0 filename=executor.py func_name=run_python_code lineno=107 stderr_length=0 stdout_length=11
PASSED
tests/worker_light/test_core.py::TestFilesystemRouter::test_read_only_utils_check PASSED
tests/worker_light/test_core.py::TestDependencyVerification::test_build123d_import 2026-02-22T10:20:53.152583Z [info     ] runtime_execute_complete       exit_code=0 filename=executor.py func_name=run_python_code lineno=107 stderr_length=0 stdout_length=7
PASSED
tests/worker_light/test_core.py::TestDependencyVerification::test_mujoco_import 2026-02-22T10:20:53.387612Z [info     ] runtime_execute_complete       exit_code=0 filename=executor.py func_name=run_python_code lineno=107 stderr_length=0 stdout_length=6
PASSED
tests/worker_light/test_filesystem.py::test_local_backend_operations PASSED
tests/worker_light/test_filesystem.py::test_local_backend_isolation PASSED
tests/worker_light/test_filesystem.py::test_filesystem_router_logic 2026-02-22T10:20:53.426927Z [warning  ] router_write_blocked           filename=router.py func_name=write lineno=248 path=/utils/new.py
PASSED
tests/worker_light/test_filesystem.py::test_router_ls_merged PASSED
tests/worker_light/test_filesystem_overwrite.py::test_local_write_overwrite PASSED
tests/worker_light/test_git_utils.py::test_git_conflict_resolution 2026-02-22T10:20:53.434142Z [info     ] initializing_git_repo          filename=git.py func_name=init_workspace_repo lineno=16 path=/tmp/pytest-of-maksym/pytest-2/test_git_conflict_resolution0
2026-02-22T10:20:53.446620Z [info     ] git_committing                 filename=git.py func_name=commit_all lineno=42 message='Initial commit' path=/tmp/pytest-of-maksym/pytest-2/test_git_conflict_resolution0
2026-02-22T10:20:53.461872Z [info     ] git_committing                 filename=git.py func_name=commit_all lineno=42 message='Feature commit' path=/tmp/pytest-of-maksym/pytest-2/test_git_conflict_resolution0
2026-02-22T10:20:53.475844Z [info     ] git_committing                 filename=git.py func_name=commit_all lineno=42 message='Main commit' path=/tmp/pytest-of-maksym/pytest-2/test_git_conflict_resolution0
2026-02-22T10:20:53.496289Z [info     ] resolved_conflict_ours         file=conflict.txt filename=git.py func_name=resolve_conflict_ours lineno=102
2026-02-22T10:20:53.507950Z [info     ] merge_completed                commit=3d47fd21a3903e9d656ff2abd09a725c034fafbb filename=git.py func_name=complete_merge lineno=176
PASSED
tests/worker_light/test_git_utils.py::test_git_abort_merge 2026-02-22T10:20:53.520981Z [info     ] initializing_git_repo          filename=git.py func_name=init_workspace_repo lineno=16 path=/tmp/pytest-of-maksym/pytest-2/test_git_abort_merge0
2026-02-22T10:20:53.528033Z [info     ] git_committing                 filename=git.py func_name=commit_all lineno=42 message='Initial commit' path=/tmp/pytest-of-maksym/pytest-2/test_git_abort_merge0
2026-02-22T10:20:53.542472Z [info     ] git_committing                 filename=git.py func_name=commit_all lineno=42 message='Feature commit' path=/tmp/pytest-of-maksym/pytest-2/test_git_abort_merge0
2026-02-22T10:20:53.556476Z [info     ] git_committing                 filename=git.py func_name=commit_all lineno=42 message='Main commit' path=/tmp/pytest-of-maksym/pytest-2/test_git_abort_merge0
2026-02-22T10:20:53.572291Z [info     ] merge_aborted                  filename=git.py func_name=abort_merge lineno=142 path=/tmp/pytest-of-maksym/pytest-2/test_git_abort_merge0
PASSED
tests/worker_light/test_router_analyzer.py::test_router_download_files PASSED
tests/worker_light/test_router_analyzer.py::test_workbench_analyzer_protocol PASSED

============================================================ ERRORS ============================================================
________________________________________ ERROR at setup of test_remote_fs_middleware_ls ________________________________________

    @pytest.fixture(autouse=True)
    def mock_observability():
>       with (
            patch(
                "controller.middleware.remote_fs.record_worker_events",
                new_callable=AsyncMock,
            ) as m1,
            patch(
                "controller.middleware.remote_fs.sync_asset", new_callable=AsyncMock
            ) as m2,
            patch(
                "controller.observability.broadcast.EpisodeBroadcaster",
                new_callable=MagicMock,
            ) as m3,
        ):

tests/controller/test_remote_fs.py:11: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
/usr/lib/python3.12/unittest/mock.py:1458: in __enter__
    original, local = self.get_original()
                      ^^^^^^^^^^^^^^^^^^^
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <unittest.mock._patch object at 0x71a5b1b2cbf0>

    def get_original(self):
        target = self.getter()
        name = self.attribute
    
        original = DEFAULT
        local = False
    
        try:
            original = target.__dict__[name]
        except (AttributeError, KeyError):
            original = getattr(target, name, DEFAULT)
        else:
            local = True
    
        if name in _builtins and isinstance(target, ModuleType):
            self.create = True
    
        if not self.create and original is DEFAULT:
>           raise AttributeError(
                "%s does not have the attribute %r" % (target, name)
            )
E           AttributeError: <module 'controller.middleware.remote_fs' from '/home/maksym/Work/proj/Problemologist/Problemologist-AI/controller/middleware/remote_fs.py'> does not have the attribute 'record_worker_events'

/usr/lib/python3.12/unittest/mock.py:1431: AttributeError
_________________________________ ERROR at setup of test_remote_fs_middleware_write_protection _________________________________

    @pytest.fixture(autouse=True)
    def mock_observability():
>       with (
            patch(
                "controller.middleware.remote_fs.record_worker_events",
                new_callable=AsyncMock,
            ) as m1,
            patch(
                "controller.middleware.remote_fs.sync_asset", new_callable=AsyncMock
            ) as m2,
            patch(
                "controller.observability.broadcast.EpisodeBroadcaster",
                new_callable=MagicMock,
            ) as m3,
        ):

tests/controller/test_remote_fs.py:11: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
/usr/lib/python3.12/unittest/mock.py:1458: in __enter__
    original, local = self.get_original()
                      ^^^^^^^^^^^^^^^^^^^
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <unittest.mock._patch object at 0x71a5b1bc1790>

    def get_original(self):
        target = self.getter()
        name = self.attribute
    
        original = DEFAULT
        local = False
    
        try:
            original = target.__dict__[name]
        except (AttributeError, KeyError):
            original = getattr(target, name, DEFAULT)
        else:
            local = True
    
        if name in _builtins and isinstance(target, ModuleType):
            self.create = True
    
        if not self.create and original is DEFAULT:
>           raise AttributeError(
                "%s does not have the attribute %r" % (target, name)
            )
E           AttributeError: <module 'controller.middleware.remote_fs' from '/home/maksym/Work/proj/Problemologist/Problemologist-AI/controller/middleware/remote_fs.py'> does not have the attribute 'record_worker_events'

/usr/lib/python3.12/unittest/mock.py:1431: AttributeError
__________________________________________ ERROR at setup of test_fs_tools_execution ___________________________________________

    @pytest.fixture(autouse=True)
    def mock_observability():
>       with (
            patch(
                "controller.middleware.remote_fs.record_worker_events",
                new_callable=AsyncMock,
            ) as m1,
            patch(
                "controller.middleware.remote_fs.sync_asset", new_callable=AsyncMock
            ) as m2,
            patch(
                "controller.observability.broadcast.EpisodeBroadcaster",
                new_callable=MagicMock,
            ) as m3,
        ):

tests/controller/test_remote_fs.py:11: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
/usr/lib/python3.12/unittest/mock.py:1458: in __enter__
    original, local = self.get_original()
                      ^^^^^^^^^^^^^^^^^^^
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <unittest.mock._patch object at 0x71a5b1b0ef30>

    def get_original(self):
        target = self.getter()
        name = self.attribute
    
        original = DEFAULT
        local = False
    
        try:
            original = target.__dict__[name]
        except (AttributeError, KeyError):
            original = getattr(target, name, DEFAULT)
        else:
            local = True
    
        if name in _builtins and isinstance(target, ModuleType):
            self.create = True
    
        if not self.create and original is DEFAULT:
>           raise AttributeError(
                "%s does not have the attribute %r" % (target, name)
            )
E           AttributeError: <module 'controller.middleware.remote_fs' from '/home/maksym/Work/proj/Problemologist/Problemologist-AI/controller/middleware/remote_fs.py'> does not have the attribute 'record_worker_events'

/usr/lib/python3.12/unittest/mock.py:1431: AttributeError
_______________________________________ ERROR at setup of test_remote_fs_middleware_read _______________________________________

    @pytest.fixture(autouse=True)
    def mock_observability():
>       with (
            patch(
                "controller.middleware.remote_fs.record_worker_events",
                new_callable=AsyncMock,
            ) as m1,
            patch(
                "controller.middleware.remote_fs.sync_asset", new_callable=AsyncMock
            ) as m2,
            patch(
                "controller.observability.broadcast.EpisodeBroadcaster",
                new_callable=MagicMock,
            ) as m3,
        ):

tests/controller/test_remote_fs.py:11: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
/usr/lib/python3.12/unittest/mock.py:1458: in __enter__
    original, local = self.get_original()
                      ^^^^^^^^^^^^^^^^^^^
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <unittest.mock._patch object at 0x71a5b1b2f8f0>

    def get_original(self):
        target = self.getter()
        name = self.attribute
    
        original = DEFAULT
        local = False
    
        try:
            original = target.__dict__[name]
        except (AttributeError, KeyError):
            original = getattr(target, name, DEFAULT)
        else:
            local = True
    
        if name in _builtins and isinstance(target, ModuleType):
            self.create = True
    
        if not self.create and original is DEFAULT:
>           raise AttributeError(
                "%s does not have the attribute %r" % (target, name)
            )
E           AttributeError: <module 'controller.middleware.remote_fs' from '/home/maksym/Work/proj/Problemologist/Problemologist-AI/controller/middleware/remote_fs.py'> does not have the attribute 'record_worker_events'

/usr/lib/python3.12/unittest/mock.py:1431: AttributeError
____________________________________ ERROR at setup of test_remote_fs_middleware_overwrite _____________________________________

    @pytest.fixture(autouse=True)
    def mock_observability():
>       with (
            patch(
                "controller.middleware.remote_fs.record_worker_events",
                new_callable=AsyncMock,
            ) as m1,
            patch(
                "controller.middleware.remote_fs.sync_asset", new_callable=AsyncMock
            ) as m2,
            patch(
                "controller.observability.broadcast.EpisodeBroadcaster",
                new_callable=MagicMock,
            ) as m3,
        ):

tests/controller/test_remote_fs.py:11: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
/usr/lib/python3.12/unittest/mock.py:1458: in __enter__
    original, local = self.get_original()
                      ^^^^^^^^^^^^^^^^^^^
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <unittest.mock._patch object at 0x71a5b1b0ef60>

    def get_original(self):
        target = self.getter()
        name = self.attribute
    
        original = DEFAULT
        local = False
    
        try:
            original = target.__dict__[name]
        except (AttributeError, KeyError):
            original = getattr(target, name, DEFAULT)
        else:
            local = True
    
        if name in _builtins and isinstance(target, ModuleType):
            self.create = True
    
        if not self.create and original is DEFAULT:
>           raise AttributeError(
                "%s does not have the attribute %r" % (target, name)
            )
E           AttributeError: <module 'controller.middleware.remote_fs' from '/home/maksym/Work/proj/Problemologist/Problemologist-AI/controller/middleware/remote_fs.py'> does not have the attribute 'record_worker_events'

/usr/lib/python3.12/unittest/mock.py:1431: AttributeError
_______________________________________ ERROR at setup of test_save_asset_with_variants ________________________________________

self = <Coroutine test_save_asset_with_variants>

    def setup(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        if runner_fixture_id not in self.fixturenames:
            self.fixturenames.append(runner_fixture_id)
>       return super().setup()
               ^^^^^^^^^^^^^^^

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:458: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:743: in pytest_fixture_setup
    hook_result = yield
                  ^^^^^
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:313: in _asyncgen_fixture_wrapper
    result = runner.run(setup(), context=context)
             ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a50826ff20>
coro = <coroutine object _wrap_asyncgen_fixture.<locals>._asyncgen_fixture_wrapper.<locals>.setup at 0x71a518525f20>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
_______________________________________ ERROR at setup of test_asset_content_persistence _______________________________________

self = <Coroutine test_asset_content_persistence>

    def setup(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        if runner_fixture_id not in self.fixturenames:
            self.fixturenames.append(runner_fixture_id)
>       return super().setup()
               ^^^^^^^^^^^^^^^

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:458: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:743: in pytest_fixture_setup
    hook_result = yield
                  ^^^^^
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:313: in _asyncgen_fixture_wrapper
    result = runner.run(setup(), context=context)
             ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a4fe7f3c80>
coro = <coroutine object _wrap_asyncgen_fixture.<locals>._asyncgen_fixture_wrapper.<locals>.setup at 0x71a5b25d7780>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
_________________________________________ ERROR at setup of test_episode_relationships _________________________________________

self = <Coroutine test_episode_relationships>

    def setup(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        if runner_fixture_id not in self.fixturenames:
            self.fixturenames.append(runner_fixture_id)
>       return super().setup()
               ^^^^^^^^^^^^^^^

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:458: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:743: in pytest_fixture_setup
    hook_result = yield
                  ^^^^^
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:313: in _asyncgen_fixture_wrapper
    result = runner.run(setup(), context=context)
             ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a4fe7f1280>
coro = <coroutine object _wrap_asyncgen_fixture.<locals>._asyncgen_fixture_wrapper.<locals>.setup at 0x71a4fc4f1970>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
=========================================================== FAILURES ===========================================================
________________________________________________ test_all_agents_initialization ________________________________________________

    @pytest.mark.asyncio
    async def test_all_agents_initialization():
        """Verify all agents can be initialized and have tools bound."""
>       with (
            patch("controller.agent.nodes.base.WorkerClient"),
            patch("controller.agent.nodes.base.RemoteFilesystemMiddleware"),
            patch(
                "controller.agent.benchmark.nodes.get_prompt", return_value="mock prompt"
            ),
        ):

tests/controller/agent/test_refactor_verification.py:22: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
/usr/lib/python3.12/unittest/mock.py:1458: in __enter__
    original, local = self.get_original()
                      ^^^^^^^^^^^^^^^^^^^
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <unittest.mock._patch object at 0x71a5b13f8cb0>

    def get_original(self):
        target = self.getter()
        name = self.attribute
    
        original = DEFAULT
        local = False
    
        try:
            original = target.__dict__[name]
        except (AttributeError, KeyError):
            original = getattr(target, name, DEFAULT)
        else:
            local = True
    
        if name in _builtins and isinstance(target, ModuleType):
            self.create = True
    
        if not self.create and original is DEFAULT:
>           raise AttributeError(
                "%s does not have the attribute %r" % (target, name)
            )
E           AttributeError: <module 'controller.agent.benchmark.nodes' from '/home/maksym/Work/proj/Problemologist/Problemologist-AI/controller/agent/benchmark/nodes.py'> does not have the attribute 'get_prompt'

/usr/lib/python3.12/unittest/mock.py:1431: AttributeError
_____________________________________________ test_metric_benchmark_planner_basic ______________________________________________

    def test_metric_benchmark_planner_basic():
        """Test using benchmark_planner milestones from reward_config.yaml"""
        gold = SimpleNamespace(
            agent_name="benchmark_planner",
            objectives=SimpleNamespace(max_unit_cost=100.0, max_weight_g=50.0),
        )
        # Give it all milestones except reviewer_accepted
        prediction = SimpleNamespace(
            plan_artifacts_present=True,
            yaml_schema_valid=True,
            cost_within_cap=True,
            estimated_cost=80.0,
            weight_within_cap=True,
            estimated_weight=40.0,
            geometry_consistent=True,
            cots_ids_valid=True,
            reviewer_accepted=False,
        )
    
        score = cad_simulation_metric(gold, prediction)
        # Weights for benchmark_planner:
        # present: 0.05
        # schema: 0.10
        # cost: 0.10
        # weight: 0.05
        # geometry: 0.10
        # cots: 0.05
        # Sum: 0.45
>       assert score == pytest.approx(0.45)
E       AssertionError: assert Prediction(\n ...sult: False'\n) == 0.45 ± 4.5e-07
E         
E         comparison failed
E         Obtained: Prediction(\n    score=0.3,\n    feedback='plan_artifacts_present result: True | yaml_schema_valid result: True | cost_within_cap score: 0.00 | weight_within_cap score: 0.00 | geometry_consistent result: True | cots_ids_valid result: True | reviewer_accepted result: False'\n)
E         Expected: 0.45 ± 4.5e-07

tests/controller/agent/test_reward_generic.py:36: AssertionError
__________________________________________ test_metric_benchmark_planner_cost_overage __________________________________________

    def test_metric_benchmark_planner_cost_overage():
        gold = SimpleNamespace(
            agent_name="benchmark_planner",
            objectives=SimpleNamespace(max_unit_cost=100.0, max_weight_g=50.0),
        )
        # 20% over cost cap -> penalty 0.8
        prediction = SimpleNamespace(
            plan_artifacts_present=True,
            yaml_schema_valid=True,
            cost_within_cap=True,
            estimated_cost=120.0,
            weight_within_cap=False,
            geometry_consistent=False,
            cots_ids_valid=False,
            reviewer_accepted=False,
        )
    
        score = cad_simulation_metric(gold, prediction)
        # present: 0.05
        # schema: 0.10
        # cost: 0.10 * 0.8 = 0.08
>       assert score == pytest.approx(0.05 + 0.10 + 0.08)
E       AssertionError: assert Prediction(\n ...sult: False'\n) == 0.23000000000000004 ± 2.3e-07
E         
E         comparison failed
E         Obtained: Prediction(\n    score=0.15000000000000002,\n    feedback='plan_artifacts_present result: True | yaml_schema_valid result: True | cost_within_cap score: 0.00 | weight_within_cap score: 0.00 | geometry_consistent result: False | cots_ids_valid result: False | reviewer_accepted result: False'\n)
E         Expected: 0.23000000000000004 ± 2.3e-07

tests/controller/agent/test_reward_generic.py:60: AssertionError
___________________________________________ test_metric_cad_engineer_failure_formula ___________________________________________

    def test_metric_cad_engineer_failure_formula():
        """Verify that cad_engineer simulation failure formula is used."""
        gold = SimpleNamespace(
            agent_name="cad_engineer",
            objectives=SimpleNamespace(max_unit_cost=10.0, max_weight_g=5.0),
        )
        prediction = SimpleNamespace(
            script_compiled=True,
            cad_geometry_valid=True,
            manufacturability_valid=True,
            parts_within_build_zone=True,
            actual_cost=5.0,
            actual_weight=2.0,
            simulation_success=False,
            simulation_ran=True,
            min_distance_to_goal=7.0,
            initial_distance=10.0,
        )
        # Base: 0.05(script) + 0.08(cad) + 0.07(mfg) + 0.05(zone) + 0.10(cost) + 0.05(weight) = 0.40
        # Failure formula: 0.60 * (1 - 7/10) * 0.4 = 0.60 * 0.3 * 0.4 = 0.072
        # Total: 0.472
        score = cad_simulation_metric(gold, prediction)
>       assert score == pytest.approx(0.472)
E       AssertionError: assert Prediction(\n ...ress: 0.12)'\n) == 0.472 ± 4.7e-07
E         
E         comparison failed
E         Obtained: Prediction(\n    score=0.322,\n    feedback='The CAD script compiled and executed without syntax or import errors. | CAD geometry is valid and successfully exported. | The design passed all manufacturability constraints (CNC/Injection Molding). | All parts are correctly positioned within the designated build zone. | Manufacturing cost exceeds the budget. Try using cheaper materials or reducing part volume. | Assembly weight exceeds the limit. Use lighter materials or hollow out solid regions. | The simulation failed to reach the goal. Re-examine mechanical sta...
E         
E         ...Full output truncated (2 lines hidden), use '-vv' to show

tests/controller/agent/test_reward_generic.py:85: AssertionError
_____________________________________________ test_metric_reviewer_generic_binary ______________________________________________

    def test_metric_reviewer_generic_binary():
        """Test benchmark_reviewer with binary milestones."""
        gold = SimpleNamespace(agent_name="benchmark_reviewer")
        prediction = SimpleNamespace(
            review_artifacts_complete=True, decision_correct=True, review_actionable=True
        )
        # reviewer weights: 0.10, 0.60, 0.30 -> Sum 1.0
        score = cad_simulation_metric(gold, prediction)
>       assert score == pytest.approx(1.0)
E       AssertionError: assert Prediction(\n ...esult: True'\n) == 1.0 ± 1.0e-06
E         
E         comparison failed
E         Obtained: Prediction(\n    score=1.0,\n    feedback='review_artifacts_complete result: True | decision_correct result: True | review_actionable result: True'\n)
E         Expected: 1.0 ± 1.0e-06

tests/controller/agent/test_reward_generic.py:96: AssertionError
___________________________________________________ test_metric_full_success ___________________________________________________

    def test_metric_full_success():
        gold = SimpleNamespace(
            agent_name="cad_engineer",
            objectives=SimpleNamespace(max_unit_cost=10.0, max_weight_g=5.0),
        )
        prediction = SimpleNamespace(
            script_compiled=True,
            cad_geometry_valid=True,
            manufacturability_valid=True,
            parts_within_build_zone=True,
            actual_cost=5.0,
            actual_weight=2.0,
            simulation_success=True,
        )
    
        result = cad_simulation_metric(gold, prediction)
        # Sum of weights: 0.05 + 0.08 + 0.07 + 0.05 + 0.10 + 0.05 + 0.60 = 1.0
>       assert result.score == 1.0
E       AssertionError: assert 0.85 == 1.0
E        +  where 0.85 = Prediction(\n    score=0.85,\n    feedback='The CAD script compiled and executed without syntax or import errors. | CAD geometry is valid and successfully exported. | The design passed all manufacturability constraints (CNC/Injection Molding). | All parts are correctly positioned within the designated build zone. | Manufacturing cost exceeds the budget. Try using cheaper materials or reducing part volume. | Assembly weight exceeds the limit. Use lighter materials or hollow out solid regions. | The solution successfully solved the problem in simulation.'\n).score

tests/controller/agent/test_reward_metric.py:49: AssertionError
___________________________________________________ test_metric_partial_sim ____________________________________________________

    def test_metric_partial_sim():
        gold = SimpleNamespace(
            agent_name="cad_engineer",
            objectives=SimpleNamespace(max_unit_cost=10.0, max_weight_g=5.0),
        )
        prediction = SimpleNamespace(
            script_compiled=True,
            cad_geometry_valid=True,
            manufacturability_valid=True,
            parts_within_build_zone=True,
            actual_cost=5.0,
            actual_weight=2.0,
            simulation_success=False,
            simulation_ran=True,
            min_distance_to_goal=5.0,
            initial_distance=10.0,
        )
    
        result = cad_simulation_metric(gold, prediction)
        # Prefixes sum: 0.05 + 0.08 + 0.07 + 0.05 + 0.10 + 0.05 = 0.40
        # Sim partial: 0.60 * (1 - 5/10) * 0.4 = 0.60 * 0.5 * 0.4 = 0.12
        # Total: 0.40 + 0.12 = 0.52
>       assert result.score == 0.52
E       AssertionError: assert 0.37 == 0.52
E        +  where 0.37 = Prediction(\n    score=0.37,\n    feedback='The CAD script compiled and executed without syntax or import errors. | CAD geometry is valid and successfully exported. | The design passed all manufacturability constraints (CNC/Injection Molding). | All parts are correctly positioned within the designated build zone. | Manufacturing cost exceeds the budget. Try using cheaper materials or reducing part volume. | Assembly weight exceeds the limit. Use lighter materials or hollow out solid regions. | The simulation failed to reach the goal. Re-examine mechanical stability, component placement, or movement range. (Progress: 0.20)'\n).score

tests/controller/agent/test_reward_metric.py:74: AssertionError
___________________________________________________ test_metric_cost_overage ___________________________________________________

    def test_metric_cost_overage():
        gold = SimpleNamespace(
            agent_name="cad_engineer",
            objectives=SimpleNamespace(max_unit_cost=10.0, max_weight_g=5.0),
        )
        prediction = SimpleNamespace(
            script_compiled=True,
            cad_geometry_valid=True,
            manufacturability_valid=True,
            parts_within_build_zone=True,
            actual_cost=15.0,  # 1.5x cap
            actual_weight=2.0,
            simulation_success=True,
        )
    
        result = cad_simulation_metric(gold, prediction)
        # Cost penalty: max(0, 1 - max(0, 1.5 - 1)) = 0.5
        # Total score: 1.0 - (0.10 * (1 - 0.5)) = 0.95
>       assert result.score == pytest.approx(0.95)
E       assert 0.85 == 0.95 ± 9.5e-07
E         
E         comparison failed
E         Obtained: 0.85
E         Expected: 0.95 ± 9.5e-07

tests/controller/agent/test_reward_metric.py:95: AssertionError
__________________________________________________ test_simulate_emits_events __________________________________________________

mock_worker_client = <MagicMock spec='WorkerClient' id='124956464709072'>

    @pytest.mark.asyncio
    async def test_simulate_emits_events(mock_worker_client):
        middleware = RemoteFilesystemMiddleware(mock_worker_client)
    
        mock_worker_client.simulate.return_value = MagicMock(
            model_dump=lambda: {
                "success": True,
                "time_elapsed_s": 1.5,
                "compute_time_ms": 100,
            }
        )
    
>       with patch(
            "controller.middleware.remote_fs.record_worker_events", new_callable=AsyncMock
        ) as mock_record:

tests/controller/test_remote_fs_events.py:53: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
/usr/lib/python3.12/unittest/mock.py:1458: in __enter__
    original, local = self.get_original()
                      ^^^^^^^^^^^^^^^^^^^
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <unittest.mock._patch object at 0x71a5b20995b0>

    def get_original(self):
        target = self.getter()
        name = self.attribute
    
        original = DEFAULT
        local = False
    
        try:
            original = target.__dict__[name]
        except (AttributeError, KeyError):
            original = getattr(target, name, DEFAULT)
        else:
            local = True
    
        if name in _builtins and isinstance(target, ModuleType):
            self.create = True
    
        if not self.create and original is DEFAULT:
>           raise AttributeError(
                "%s does not have the attribute %r" % (target, name)
            )
E           AttributeError: <module 'controller.middleware.remote_fs' from '/home/maksym/Work/proj/Problemologist/Problemologist-AI/controller/middleware/remote_fs.py'> does not have the attribute 'record_worker_events'

/usr/lib/python3.12/unittest/mock.py:1431: AttributeError
__________________________________________________ test_validate_emits_events __________________________________________________

mock_worker_client = <MagicMock spec='WorkerClient' id='124956470509888'>

    @pytest.mark.asyncio
    async def test_validate_emits_events(mock_worker_client):
        middleware = RemoteFilesystemMiddleware(mock_worker_client)
    
        mock_worker_client.validate.return_value = MagicMock(
            model_dump=lambda: {"success": True, "price": 50.0}
        )
    
>       with patch(
            "controller.middleware.remote_fs.record_worker_events", new_callable=AsyncMock
        ) as mock_record:

tests/controller/test_remote_fs_events.py:79: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
/usr/lib/python3.12/unittest/mock.py:1458: in __enter__
    original, local = self.get_original()
                      ^^^^^^^^^^^^^^^^^^^
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <unittest.mock._patch object at 0x71a5b2079a00>

    def get_original(self):
        target = self.getter()
        name = self.attribute
    
        original = DEFAULT
        local = False
    
        try:
            original = target.__dict__[name]
        except (AttributeError, KeyError):
            original = getattr(target, name, DEFAULT)
        else:
            local = True
    
        if name in _builtins and isinstance(target, ModuleType):
            self.create = True
    
        if not self.create and original is DEFAULT:
>           raise AttributeError(
                "%s does not have the attribute %r" % (target, name)
            )
E           AttributeError: <module 'controller.middleware.remote_fs' from '/home/maksym/Work/proj/Problemologist/Problemologist-AI/controller/middleware/remote_fs.py'> does not have the attribute 'record_worker_events'

/usr/lib/python3.12/unittest/mock.py:1431: AttributeError
___________________________________________________ test_submit_emits_events ___________________________________________________

mock_worker_client = <MagicMock spec='WorkerClient' id='124956470389040'>

    @pytest.mark.asyncio
    async def test_submit_emits_events(mock_worker_client):
        middleware = RemoteFilesystemMiddleware(mock_worker_client)
    
        mock_worker_client.submit.return_value = MagicMock(
            model_dump=lambda: {"success": True}
        )
    
>       with patch(
            "controller.middleware.remote_fs.record_worker_events", new_callable=AsyncMock
        ) as mock_record:

tests/controller/test_remote_fs_events.py:99: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
/usr/lib/python3.12/unittest/mock.py:1458: in __enter__
    original, local = self.get_original()
                      ^^^^^^^^^^^^^^^^^^^
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <unittest.mock._patch object at 0x71a5b1b134d0>

    def get_original(self):
        target = self.getter()
        name = self.attribute
    
        original = DEFAULT
        local = False
    
        try:
            original = target.__dict__[name]
        except (AttributeError, KeyError):
            original = getattr(target, name, DEFAULT)
        else:
            local = True
    
        if name in _builtins and isinstance(target, ModuleType):
            self.create = True
    
        if not self.create and original is DEFAULT:
>           raise AttributeError(
                "%s does not have the attribute %r" % (target, name)
            )
E           AttributeError: <module 'controller.middleware.remote_fs' from '/home/maksym/Work/proj/Problemologist/Problemologist-AI/controller/middleware/remote_fs.py'> does not have the attribute 'record_worker_events'

/usr/lib/python3.12/unittest/mock.py:1431: AttributeError
__________________________________________________ test_ls_files_emits_event ___________________________________________________

mock_worker_client = <MagicMock spec='WorkerClient' id='124956470383280'>

    @pytest.mark.asyncio
    async def test_ls_files_emits_event(mock_worker_client):
        middleware = RemoteFilesystemMiddleware(mock_worker_client)
        mock_worker_client.list_files.return_value = []
    
>       with patch(
            "controller.middleware.remote_fs.record_worker_events", new_callable=AsyncMock
        ) as mock_record:

tests/controller/test_remote_fs_events.py:116: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
/usr/lib/python3.12/unittest/mock.py:1458: in __enter__
    original, local = self.get_original()
                      ^^^^^^^^^^^^^^^^^^^
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <unittest.mock._patch object at 0x71a5b209b980>

    def get_original(self):
        target = self.getter()
        name = self.attribute
    
        original = DEFAULT
        local = False
    
        try:
            original = target.__dict__[name]
        except (AttributeError, KeyError):
            original = getattr(target, name, DEFAULT)
        else:
            local = True
    
        if name in _builtins and isinstance(target, ModuleType):
            self.create = True
    
        if not self.create and original is DEFAULT:
>           raise AttributeError(
                "%s does not have the attribute %r" % (target, name)
            )
E           AttributeError: <module 'controller.middleware.remote_fs' from '/home/maksym/Work/proj/Problemologist/Problemologist-AI/controller/middleware/remote_fs.py'> does not have the attribute 'record_worker_events'

/usr/lib/python3.12/unittest/mock.py:1431: AttributeError
_________________________________________________ test_read_file_emits_events __________________________________________________

mock_worker_client = <MagicMock spec='WorkerClient' id='124956470522224'>

    @pytest.mark.asyncio
    async def test_read_file_emits_events(mock_worker_client):
        middleware = RemoteFilesystemMiddleware(mock_worker_client)
        mock_worker_client.read_file.return_value = "content"
    
        # Test regular file
>       with patch(
            "controller.middleware.remote_fs.record_worker_events", new_callable=AsyncMock
        ) as mock_record:

tests/controller/test_remote_fs_events.py:131: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
/usr/lib/python3.12/unittest/mock.py:1458: in __enter__
    original, local = self.get_original()
                      ^^^^^^^^^^^^^^^^^^^
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <unittest.mock._patch object at 0x71a5b1bc1f10>

    def get_original(self):
        target = self.getter()
        name = self.attribute
    
        original = DEFAULT
        local = False
    
        try:
            original = target.__dict__[name]
        except (AttributeError, KeyError):
            original = getattr(target, name, DEFAULT)
        else:
            local = True
    
        if name in _builtins and isinstance(target, ModuleType):
            self.create = True
    
        if not self.create and original is DEFAULT:
>           raise AttributeError(
                "%s does not have the attribute %r" % (target, name)
            )
E           AttributeError: <module 'controller.middleware.remote_fs' from '/home/maksym/Work/proj/Problemologist/Problemologist-AI/controller/middleware/remote_fs.py'> does not have the attribute 'record_worker_events'

/usr/lib/python3.12/unittest/mock.py:1431: AttributeError
_________________________________________________ test_write_file_emits_event __________________________________________________

mock_worker_client = <MagicMock spec='WorkerClient' id='124956464708400'>

    @pytest.mark.asyncio
    async def test_write_file_emits_event(mock_worker_client):
        middleware = RemoteFilesystemMiddleware(mock_worker_client)
        mock_worker_client.write_file.return_value = True
    
>       with patch(
            "controller.middleware.remote_fs.record_worker_events", new_callable=AsyncMock
        ) as mock_record:

tests/controller/test_remote_fs_events.py:165: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
/usr/lib/python3.12/unittest/mock.py:1458: in __enter__
    original, local = self.get_original()
                      ^^^^^^^^^^^^^^^^^^^
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <unittest.mock._patch object at 0x71a5b1357590>

    def get_original(self):
        target = self.getter()
        name = self.attribute
    
        original = DEFAULT
        local = False
    
        try:
            original = target.__dict__[name]
        except (AttributeError, KeyError):
            original = getattr(target, name, DEFAULT)
        else:
            local = True
    
        if name in _builtins and isinstance(target, ModuleType):
            self.create = True
    
        if not self.create and original is DEFAULT:
>           raise AttributeError(
                "%s does not have the attribute %r" % (target, name)
            )
E           AttributeError: <module 'controller.middleware.remote_fs' from '/home/maksym/Work/proj/Problemologist/Problemologist-AI/controller/middleware/remote_fs.py'> does not have the attribute 'record_worker_events'

/usr/lib/python3.12/unittest/mock.py:1431: AttributeError
__________________________________________________ test_edit_file_emits_event __________________________________________________

mock_worker_client = <MagicMock spec='WorkerClient' id='124956456612016'>

    @pytest.mark.asyncio
    async def test_edit_file_emits_event(mock_worker_client):
        middleware = RemoteFilesystemMiddleware(mock_worker_client)
        mock_worker_client.edit_file.return_value = True
    
>       with patch(
            "controller.middleware.remote_fs.record_worker_events", new_callable=AsyncMock
        ) as mock_record:

tests/controller/test_remote_fs_events.py:181: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
/usr/lib/python3.12/unittest/mock.py:1458: in __enter__
    original, local = self.get_original()
                      ^^^^^^^^^^^^^^^^^^^
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <unittest.mock._patch object at 0x71a5b200d010>

    def get_original(self):
        target = self.getter()
        name = self.attribute
    
        original = DEFAULT
        local = False
    
        try:
            original = target.__dict__[name]
        except (AttributeError, KeyError):
            original = getattr(target, name, DEFAULT)
        else:
            local = True
    
        if name in _builtins and isinstance(target, ModuleType):
            self.create = True
    
        if not self.create and original is DEFAULT:
>           raise AttributeError(
                "%s does not have the attribute %r" % (target, name)
            )
E           AttributeError: <module 'controller.middleware.remote_fs' from '/home/maksym/Work/proj/Problemologist/Problemologist-AI/controller/middleware/remote_fs.py'> does not have the attribute 'record_worker_events'

/usr/lib/python3.12/unittest/mock.py:1431: AttributeError
____________________________________________________ test_grep_emits_event _____________________________________________________

mock_worker_client = <MagicMock spec='WorkerClient' id='124956456600256'>

    @pytest.mark.asyncio
    async def test_grep_emits_event(mock_worker_client):
        middleware = RemoteFilesystemMiddleware(mock_worker_client)
        mock_worker_client.grep.return_value = []
    
>       with patch(
            "controller.middleware.remote_fs.record_worker_events", new_callable=AsyncMock
        ) as mock_record:

tests/controller/test_remote_fs_events.py:195: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
/usr/lib/python3.12/unittest/mock.py:1458: in __enter__
    original, local = self.get_original()
                      ^^^^^^^^^^^^^^^^^^^
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <unittest.mock._patch object at 0x71a5b1bc21e0>

    def get_original(self):
        target = self.getter()
        name = self.attribute
    
        original = DEFAULT
        local = False
    
        try:
            original = target.__dict__[name]
        except (AttributeError, KeyError):
            original = getattr(target, name, DEFAULT)
        else:
            local = True
    
        if name in _builtins and isinstance(target, ModuleType):
            self.create = True
    
        if not self.create and original is DEFAULT:
>           raise AttributeError(
                "%s does not have the attribute %r" % (target, name)
            )
E           AttributeError: <module 'controller.middleware.remote_fs' from '/home/maksym/Work/proj/Problemologist/Problemologist-AI/controller/middleware/remote_fs.py'> does not have the attribute 'record_worker_events'

/usr/lib/python3.12/unittest/mock.py:1431: AttributeError
_________________________________________________ test_run_command_emits_event _________________________________________________

mock_worker_client = <MagicMock spec='WorkerClient' id='124956465438672'>

    @pytest.mark.asyncio
    async def test_run_command_emits_event(mock_worker_client):
        middleware = RemoteFilesystemMiddleware(mock_worker_client)
        mock_worker_client.execute_python.return_value = MagicMock(model_dump=lambda: {})
    
>       with patch(
            "controller.middleware.remote_fs.record_worker_events", new_callable=AsyncMock
        ) as mock_record:

tests/controller/test_remote_fs_events.py:209: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
/usr/lib/python3.12/unittest/mock.py:1458: in __enter__
    original, local = self.get_original()
                      ^^^^^^^^^^^^^^^^^^^
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <unittest.mock._patch object at 0x71a5b209b830>

    def get_original(self):
        target = self.getter()
        name = self.attribute
    
        original = DEFAULT
        local = False
    
        try:
            original = target.__dict__[name]
        except (AttributeError, KeyError):
            original = getattr(target, name, DEFAULT)
        else:
            local = True
    
        if name in _builtins and isinstance(target, ModuleType):
            self.create = True
    
        if not self.create and original is DEFAULT:
>           raise AttributeError(
                "%s does not have the attribute %r" % (target, name)
            )
E           AttributeError: <module 'controller.middleware.remote_fs' from '/home/maksym/Work/proj/Problemologist/Problemologist-AI/controller/middleware/remote_fs.py'> does not have the attribute 'record_worker_events'

/usr/lib/python3.12/unittest/mock.py:1431: AttributeError
____________________________________________________ test_cots_search_event ____________________________________________________

    def test_cots_search_event():
        # We need a db path, let's mock search_parts or use a dummy file
        # For now, let's just mock the emission if needed or use a real call if possible
        # Actually, search_parts hits a real sqlite db.
    
        # Let's try to just call it with a non-existent db and catch the error if it still emits
        # wait, search_parts emits event AFTER search.
    
        clear_emitted_events()
        query = SearchQuery(query="motor", limit=5)
    
        # We might need to mock create_engine or just provide a dummy db
        # To keep it simple, I'll just verify the emission logic exists.
        try:
            search_parts(query, "non_existent.db")
        except Exception:
            pass
    
        # Even if it fails, it might not reach emit_event.
        # Actually looking at shared/cots/runtime.py:67, it emits AFTER stmt.all()
        # So we need it to succeed or mock it.
    
        from unittest.mock import patch
    
        with patch("shared.cots.runtime.Session"):
>           search_parts(query, "dummy.db")

tests/cots/test_cots_events.py:49: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

query = SearchQuery(query='motor', constraints=None, limit=5), db_path = 'dummy.db'

    def search_parts(query: SearchQuery, db_path: str) -> list[COTSItem]:
        """
        Search for COTS parts in the database based on a query and constraints.
        """
        engine = create_engine(f"sqlite:///{db_path}")
        results = []
    
        with Session(engine) as session:
            stmt = session.query(COTSItemORM)
    
            # Text query matching name or category
            if query.query:
                search_str = f"%{query.query}%"
                stmt = stmt.filter(
                    or_(
                        COTSItemORM.name.ilike(search_str),
                        COTSItemORM.category.ilike(search_str),
                    )
                )
    
            # Apply constraints from the query
            if query.constraints:
                if "max_weight_g" in query.constraints:
                    stmt = stmt.filter(
                        COTSItemORM.weight_g <= float(query.constraints["max_weight_g"])
                    )
                if "max_cost" in query.constraints:
                    stmt = stmt.filter(
                        COTSItemORM.unit_cost <= float(query.constraints["max_cost"])
                    )
                if "category" in query.constraints:
                    stmt = stmt.filter(
                        COTSItemORM.category == query.constraints["category"]
                    )
                if "min_size" in query.constraints:
                    # Assuming metadata contains bbox or params.size
                    # This is more complex to filter in SQL directly if it's in JSON
                    # For MVP, we might skip complex JSON filtering or do it in-memory
                    pass
    
            # Limit results
            stmt = stmt.limit(query.limit)
    
            orm_items = stmt.all()
            for item in orm_items:
                results.append(
                    COTSItem(
                        part_id=item.part_id,
                        name=item.name,
                        category=item.category,
                        unit_cost=item.unit_cost,
                        weight_g=item.weight_g,
                        import_recipe=item.import_recipe,
                        metadata=item.metadata_dict,
                    )
                )
    
        # Fetch catalog metadata for reproducibility
        catalog_version = None
        bd_warehouse_commit = None
        generated_at = None
    
        with Session(engine) as session:
            meta_stmt = (
                session.query(CatalogMetadataORM)
                .order_by(CatalogMetadataORM.id.desc())
                .limit(1)
            )
            meta_result = meta_stmt.first()
            if meta_result:
                catalog_version = meta_result.catalog_version
                bd_warehouse_commit = meta_result.bd_warehouse_commit
                generated_at = (
                    meta_result.generated_at.isoformat()
                    if meta_result.generated_at
                    else None
                )
    
        # Emit search event
        emit_event(
>           COTSSearchEvent(
                query=query.query or str(query.constraints),
                results_count=len(results),
                catalog_version=catalog_version,
                bd_warehouse_commit=bd_warehouse_commit,
                generated_at=generated_at,
            )
        )
E       pydantic_core._pydantic_core.ValidationError: 3 validation errors for COTSSearchEvent
E       catalog_version
E         Input should be a valid string [type=string_type, input_value=<MagicMock name='Session(...n' id='124956446691296'>, input_type=MagicMock]
E           For further information visit https://errors.pydantic.dev/2.12/v/string_type
E       bd_warehouse_commit
E         Input should be a valid string [type=string_type, input_value=<MagicMock name='Session(...t' id='124956179325744'>, input_type=MagicMock]
E           For further information visit https://errors.pydantic.dev/2.12/v/string_type
E       generated_at
E         Input should be a valid string [type=string_type, input_value=<MagicMock name='Session(...)' id='124956179340976'>, input_type=MagicMock]
E           For further information visit https://errors.pydantic.dev/2.12/v/string_type

shared/cots/runtime.py:91: ValidationError
______________________________________________________ test_indexer_basic ______________________________________________________

tmp_path = PosixPath('/tmp/pytest-of-maksym/pytest-2/test_indexer_basic0')

    def test_indexer_basic(tmp_path):
        db_path = tmp_path / "test_parts.db"
        indexer = Indexer(str(db_path))
    
        # Index only 1 item per class to be fast
        indexer.index_all(limit_per_class=1)
    
        assert db_path.exists()
    
        engine = create_engine(f"sqlite:///{db_path}")
        with Session(engine) as session:
            items = session.query(COTSItemORM).all()
            # We expect 9 classes * 1 item = 9 items
>           assert len(items) == 9
E           assert 10 == 9
E            +  where 10 = len([<shared.cots.database.models.COTSItemORM object at 0x71a5a0b39340>, <shared.cots.database.models.COTSItemORM object at 0x71a5a0b39b80>, <shared.cots.database.models.COTSItemORM object at 0x71a5a0b38f80>, <shared.cots.database.models.COTSItemORM object at 0x71a5a0b38f50>, <shared.cots.database.models.COTSItemORM object at 0x71a5a0b38f20>, <shared.cots.database.models.COTSItemORM object at 0x71a5a0b38ef0>, ...])

tests/cots/test_indexer.py:21: AssertionError
------------------------------------------------------ Captured log call -------------------------------------------------------
INFO     shared.cots.indexer:indexer.py:159 Indexing HexNut...
INFO     build123d:build_common.py:345 None context requested by None
INFO     build123d:build_common.py:279 Entering BuildSketch with mode=Mode.ADD which is in different scope as parent
INFO     build123d:build_common.py:1275 WorkplaneList is pushing 1 workplanes: [Plane(o=(0.00, 0.00, 0.00), x=(1.00, 0.00, 0.00), z=(0.00, -1.00, 0.00))]
INFO     build123d:build_common.py:887 LocationList is pushing 1 points: [(p=(0.00, 0.00, 0.00), o=(-0.00, 0.00, -0.00))]
INFO     build123d:build_common.py:345 BuildSketch context requested by Polygon
INFO     build123d:build_common.py:491 Completed integrating 1 object(s) into part with Mode=Mode.ADD
INFO     build123d:build_common.py:898 LocationList is popping 1 points
INFO     build123d:build_common.py:1288 WorkplaneList is popping 1 workplanes
INFO     build123d:build_common.py:321 Exiting BuildSketch
INFO     build123d:build_common.py:345 None context requested by revolve
INFO     build123d:build_common.py:345 None context requested by None
INFO     build123d:build_common.py:279 Entering BuildSketch with mode=Mode.ADD which is in different scope as parent
INFO     build123d:build_common.py:1275 WorkplaneList is pushing 1 workplanes: [Plane(o=(0.00, 0.00, 0.00), x=(1.00, 0.00, 0.00), z=(0.00, 0.00, 1.00))]
INFO     build123d:build_common.py:887 LocationList is pushing 1 points: [(p=(0.00, 0.00, 0.00), o=(-0.00, 0.00, -0.00))]
INFO     build123d:build_common.py:345 BuildSketch context requested by RegularPolygon
INFO     build123d:build_common.py:491 Completed integrating 1 object(s) into part with Mode=Mode.ADD
INFO     build123d:build_common.py:898 LocationList is popping 1 points
INFO     build123d:build_common.py:1288 WorkplaneList is popping 1 workplanes
INFO     build123d:build_common.py:321 Exiting BuildSketch
INFO     build123d:build_common.py:345 None context requested by extrude
INFO     build123d:operations_part.py:187 1 face(s) to extrude on 1 face plane(s)
INFO     build123d:build_common.py:345 None context requested by None
INFO     build123d:build_common.py:345 None context requested by None
INFO     shared.cots.indexer:indexer.py:207 Indexed 1 items for HexNut
INFO     shared.cots.indexer:indexer.py:159 Indexing SocketHeadCapScrew...
INFO     build123d:build_common.py:345 None context requested by None
INFO     build123d:build_common.py:279 Entering BuildSketch with mode=Mode.ADD which is in different scope as parent
INFO     build123d:build_common.py:1275 WorkplaneList is pushing 1 workplanes: [Plane(o=(0.00, 0.00, 0.00), x=(1.00, 0.00, 0.00), z=(0.00, -1.00, 0.00))]
INFO     build123d:build_common.py:887 LocationList is pushing 1 points: [(p=(0.00, 0.00, 0.00), o=(-0.00, 0.00, -0.00))]
INFO     build123d:build_common.py:345 BuildSketch context requested by Rectangle
INFO     build123d:build_common.py:491 Completed integrating 1 object(s) into part with Mode=Mode.ADD
INFO     build123d:build_common.py:345 BuildSketch context requested by fillet
INFO     build123d:build_common.py:491 Completed integrating 1 object(s) into part with Mode=Mode.REPLACE
INFO     build123d:build_common.py:898 LocationList is popping 1 points
INFO     build123d:build_common.py:1288 WorkplaneList is popping 1 workplanes
INFO     build123d:build_common.py:321 Exiting BuildSketch
INFO     build123d:build_common.py:345 None context requested by revolve
INFO     build123d:build_common.py:345 None context requested by None
INFO     build123d:build_common.py:279 Entering BuildSketch with mode=Mode.ADD which is in different scope as parent
INFO     build123d:build_common.py:1275 WorkplaneList is pushing 1 workplanes: [Plane(o=(0.00, 0.00, 0.00), x=(1.00, 0.00, 0.00), z=(0.00, 0.00, 1.00))]
INFO     build123d:build_common.py:887 LocationList is pushing 1 points: [(p=(0.00, 0.00, 0.00), o=(-0.00, 0.00, -0.00))]
INFO     build123d:build_common.py:345 BuildSketch context requested by RegularPolygon
INFO     build123d:build_common.py:491 Completed integrating 1 object(s) into part with Mode=Mode.ADD
INFO     build123d:build_common.py:898 LocationList is popping 1 points
INFO     build123d:build_common.py:1288 WorkplaneList is popping 1 workplanes
INFO     build123d:build_common.py:321 Exiting BuildSketch
INFO     build123d:build_common.py:345 None context requested by extrude
INFO     build123d:operations_part.py:187 1 face(s) to extrude on 1 face plane(s)
INFO     build123d:build_common.py:345 None context requested by None
INFO     shared.cots.indexer:indexer.py:207 Indexed 1 items for SocketHeadCapScrew
INFO     shared.cots.indexer:indexer.py:159 Indexing PlainWasher...
INFO     build123d:build_common.py:345 None context requested by None
INFO     build123d:build_common.py:279 Entering BuildSketch with mode=Mode.ADD which is in different scope as parent
INFO     build123d:build_common.py:1275 WorkplaneList is pushing 1 workplanes: [Plane(o=(0.00, 0.00, 0.00), x=(1.00, 0.00, 0.00), z=(0.00, -1.00, 0.00))]
INFO     build123d:build_common.py:887 LocationList is pushing 1 points: [(p=(0.00, 0.00, 0.00), o=(-0.00, 0.00, -0.00))]
INFO     build123d:build_common.py:887 Locations is pushing 1 points: [(p=(0.85, 0.00, 0.00), o=(-0.00, 0.00, -0.00))]
INFO     build123d:build_common.py:345 BuildSketch context requested by Rectangle
INFO     build123d:build_common.py:491 Completed integrating 1 object(s) into part with Mode=Mode.ADD
INFO     build123d:build_common.py:898 Locations is popping 1 points
INFO     build123d:build_common.py:898 LocationList is popping 1 points
INFO     build123d:build_common.py:1288 WorkplaneList is popping 1 workplanes
INFO     build123d:build_common.py:321 Exiting BuildSketch
INFO     build123d:build_common.py:345 None context requested by revolve
INFO     shared.cots.indexer:indexer.py:207 Indexed 1 items for PlainWasher
INFO     shared.cots.indexer:indexer.py:159 Indexing SingleRowDeepGrooveBallBearing...
INFO     build123d:build_common.py:345 None context requested by None
INFO     build123d:build_common.py:279 Entering BuildSketch with mode=Mode.ADD which is in different scope as parent
INFO     build123d:build_common.py:1275 WorkplaneList is pushing 1 workplanes: [Plane(o=(0.00, 0.00, 0.00), x=(1.00, 0.00, 0.00), z=(0.00, -1.00, 0.00))]
INFO     build123d:build_common.py:887 LocationList is pushing 1 points: [(p=(0.00, 0.00, 0.00), o=(-0.00, 0.00, -0.00))]
INFO     build123d:build_common.py:887 Locations is pushing 1 points: [(p=(4.38, 0.00, 0.00), o=(-0.00, 0.00, -0.00))]
INFO     build123d:build_common.py:345 BuildSketch context requested by RectangleRounded
INFO     build123d:build_common.py:491 Completed integrating 1 object(s) into part with Mode=Mode.ADD
INFO     build123d:build_common.py:898 Locations is popping 1 points
INFO     build123d:build_common.py:898 LocationList is popping 1 points
INFO     build123d:build_common.py:1288 WorkplaneList is popping 1 workplanes
INFO     build123d:build_common.py:321 Exiting BuildSketch
INFO     build123d:build_common.py:345 None context requested by revolve
INFO     build123d:build_common.py:345 None context requested by None
INFO     build123d:build_common.py:279 Entering BuildSketch with mode=Mode.ADD which is in different scope as parent
INFO     build123d:build_common.py:1275 WorkplaneList is pushing 1 workplanes: [Plane(o=(0.00, 0.00, 0.00), x=(1.00, 0.00, 0.00), z=(0.00, -1.00, 0.00))]
INFO     build123d:build_common.py:887 LocationList is pushing 1 points: [(p=(0.00, 0.00, 0.00), o=(-0.00, 0.00, -0.00))]
INFO     build123d:build_common.py:887 Locations is pushing 1 points: [(p=(2.05, 0.00, 0.00), o=(-0.00, 0.00, -0.00))]
INFO     build123d:build_common.py:345 BuildSketch context requested by RectangleRounded
INFO     build123d:build_common.py:491 Completed integrating 1 object(s) into part with Mode=Mode.ADD
INFO     build123d:build_common.py:898 Locations is popping 1 points
INFO     build123d:build_common.py:898 LocationList is popping 1 points
INFO     build123d:build_common.py:1288 WorkplaneList is popping 1 workplanes
INFO     build123d:build_common.py:321 Exiting BuildSketch
INFO     build123d:build_common.py:345 None context requested by revolve
INFO     build123d:build_common.py:345 None context requested by None
INFO     build123d:build_common.py:345 None context requested by None
INFO     shared.cots.indexer:indexer.py:207 Indexed 1 items for SingleRowDeepGrooveBallBearing
INFO     shared.cots.indexer:indexer.py:159 Indexing ServoMotor...
INFO     build123d:build_common.py:345 None context requested by Box
INFO     shared.cots.indexer:indexer.py:207 Indexed 1 items for ServoMotor
INFO     shared.cots.indexer:indexer.py:159 Indexing PowerSupply...
INFO     build123d:build_common.py:345 None context requested by Box
INFO     shared.cots.indexer:indexer.py:207 Indexed 1 items for PowerSupply
INFO     shared.cots.indexer:indexer.py:159 Indexing ElectronicRelay...
INFO     build123d:build_common.py:345 None context requested by Box
INFO     shared.cots.indexer:indexer.py:207 Indexed 1 items for ElectronicRelay
INFO     shared.cots.indexer:indexer.py:159 Indexing Switch...
INFO     build123d:build_common.py:345 None context requested by Box
INFO     shared.cots.indexer:indexer.py:207 Indexed 1 items for Switch
INFO     shared.cots.indexer:indexer.py:159 Indexing Connector...
INFO     build123d:build_common.py:345 None context requested by Box
INFO     shared.cots.indexer:indexer.py:207 Indexed 1 items for Connector
INFO     shared.cots.indexer:indexer.py:159 Indexing Wire...
INFO     build123d:build_common.py:345 None context requested by Box
INFO     shared.cots.indexer:indexer.py:207 Indexed 1 items for Wire
INFO     shared.cots.indexer:indexer.py:216 Catalog metadata stored.
________________________________________________ test_search_cots_catalog_tool _________________________________________________

mock_parts = [COTSItem(part_id='test_bolt_m6', name='M6 Hex Bolt', category=<COTSCategory.FASTENER: 'fastener'>, unit_cost=0.5, wei...g=15.0, import_recipe="from bd_warehouse.fastener import HexBolt\nbolt = HexBolt(size='M6')", metadata={'size': 'M6'})]

    def test_search_cots_catalog_tool(mock_parts):
        with patch("shared.cots.agent.search_parts") as mock_search:
            mock_search.return_value = mock_parts
    
            # Tools in LangChain are invoked via .invoke()
>           result = search_cots_catalog.invoke({"query": "bolt", "max_weight_g": 20.0})
                     ^^^^^^^^^^^^^^^^^^^^^^^^^^
E           AttributeError: 'function' object has no attribute 'invoke'

tests/cots/test_search_agent.py:30: AttributeError
_____________________________________________ test_search_cots_catalog_no_results ______________________________________________

    def test_search_cots_catalog_no_results():
        with patch("shared.cots.agent.search_parts") as mock_search:
            mock_search.return_value = []
    
>           result = search_cots_catalog.invoke({"query": "nonexistent"})
                     ^^^^^^^^^^^^^^^^^^^^^^^^^^
E           AttributeError: 'function' object has no attribute 'invoke'

tests/cots/test_search_agent.py:47: AttributeError
________________________________________________ test_create_cots_search_agent _________________________________________________

    def test_create_cots_search_agent():
>       with patch("shared.cots.agent.ChatOpenAI") as mock_chat:

tests/cots/test_search_agent.py:53: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
/usr/lib/python3.12/unittest/mock.py:1458: in __enter__
    original, local = self.get_original()
                      ^^^^^^^^^^^^^^^^^^^
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <unittest.mock._patch object at 0x71a5b095c0e0>

    def get_original(self):
        target = self.getter()
        name = self.attribute
    
        original = DEFAULT
        local = False
    
        try:
            original = target.__dict__[name]
        except (AttributeError, KeyError):
            original = getattr(target, name, DEFAULT)
        else:
            local = True
    
        if name in _builtins and isinstance(target, ModuleType):
            self.create = True
    
        if not self.create and original is DEFAULT:
>           raise AttributeError(
                "%s does not have the attribute %r" % (target, name)
            )
E           AttributeError: <module 'shared.cots.agent' from '/home/maksym/Work/proj/Problemologist/Problemologist-AI/shared/cots/agent.py'> does not have the attribute 'ChatOpenAI'

/usr/lib/python3.12/unittest/mock.py:1431: AttributeError
____________________________________________ test_benchmark_creation_flow[chromium] ____________________________________________

page = <Page url='about:blank'>

    @pytest.mark.integration_playwright
    def test_benchmark_creation_flow(page: Page):
        # 1. Navigate to the local development server
>       page.goto("http://localhost:5173", timeout=60000)

tests/e2e/test_playwright_benchmark.py:8: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/playwright/sync_api/_generated.py:9054: in goto
    self._sync(
.venv/lib/python3.12/site-packages/playwright/_impl/_page.py:552: in goto
    return await self._main_frame.goto(**locals_to_params(locals()))
           ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.venv/lib/python3.12/site-packages/playwright/_impl/_frame.py:153: in goto
    await self._channel.send(
.venv/lib/python3.12/site-packages/playwright/_impl/_connection.py:69: in send
    return await self._connection.wrap_api_call(
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <playwright._impl._connection.Connection object at 0x71a5a0b8ad20>
cb = <function Channel.send.<locals>.<lambda> at 0x71a5a0112ca0>, is_internal = False, title = None

    async def wrap_api_call(
        self, cb: Callable[[], Any], is_internal: bool = False, title: str = None
    ) -> Any:
        if self._api_zone.get():
            return await cb()
        task = asyncio.current_task(self._loop)
        st: List[inspect.FrameInfo] = getattr(
            task, "__pw_stack__", None
        ) or inspect.stack(0)
    
        parsed_st = _extract_stack_trace_information_from_stack(st, is_internal, title)
        self._api_zone.set(parsed_st)
        try:
            return await cb()
        except Exception as error:
>           raise rewrite_error(error, f"{parsed_st['apiName']}: {error}") from None
E           playwright._impl._errors.Error: Page.goto: net::ERR_CONNECTION_REFUSED at http://localhost:5173/
E           Call log:
E             - navigating to "http://localhost:5173/", waiting until "load"

.venv/lib/python3.12/site-packages/playwright/_impl/_connection.py:559: Error
________________________________________________ test_int_126_wire_tear_failure ________________________________________________

monkeypatch = <_pytest.monkeypatch.MonkeyPatch object at 0x71a590426390>

    def test_int_126_wire_tear_failure(monkeypatch):
        """INT-126: Simulation fails if wire tension exceeds limit."""
        psu_config = PowerSupplyConfig(voltage_dc=12.0, max_current_a=10.0)
        electronics = ElectronicsSection(
            power_supply=psu_config,
            components=[ElectronicComponent(component_id="m1", type="motor")],
            wiring=[
                WireConfig(
                    wire_id="wire_torn_test",
                    from_terminal=WireTerminal(component="supply", terminal="v+"),
                    to_terminal=WireTerminal(component="m1", terminal="+"),
                    gauge_awg=24,
                    length_mm=100.0,
                    routed_in_3d=True,
                ),
                WireConfig(
                    wire_id="w2",
                    from_terminal=WireTerminal(component="m1", terminal="-"),
                    to_terminal=WireTerminal(component="supply", terminal="0"),
                    gauge_awg=24,
                    length_mm=100.0,
                    routed_in_3d=True,
                ),
            ],
        )
    
        loop = SimulationLoop(
            xml_path="tests/assets/empty_scene.xml", electronics=electronics
        )
    
        # Mock backend to return high tension
        def mock_get_tendon_tension(wire_id):
            if wire_id == "wire_torn_test":
                return 1000.0  # Way over limit for AWG24
            return 0.0
    
        monkeypatch.setattr(loop.backend, "get_tendon_tension", mock_get_tendon_tension)
    
        metrics = loop.step(control_inputs={}, duration=0.01)
    
        assert metrics.success is False
>       assert "wire_torn:wire_torn_test" in metrics.fail_reason
E       AssertionError: assert 'wire_torn:wire_torn_test' in 'WIRE_TORN:wire_torn_test'
E        +  where 'WIRE_TORN:wire_torn_test' = SimulationMetrics(total_time=0.002, total_energy=0.0, max_velocity=0.0, max_stress=0.0, success=False, fail_reason='WIRE_TORN:wire_torn_test', fail_mode=<FailureReason.WIRE_TORN: 'WIRE_TORN'>, failure=SimulationFailure(reason=<FailureReason.WIRE_TORN: 'WIRE_TORN'>, detail='wire_torn_test'), stress_summaries=[], stress_fields={}, fluid_metrics=[], events=[], confidence='approximate').fail_reason

tests/electronics/test_integration_electronics.py:142: AssertionError
------------------------------------------------------ Captured log call -------------------------------------------------------
WARNING  genesis:logger.py:131 Using a simulation timestep smaller than 2ms is not recommended for 'use_gjk_collision=False' as it could lead to numerically unstable collision detection.
INFO     genesis:logger.py:127 Scene ~~~<<a347606>>~~~ created.
INFO     genesis:logger.py:127 Adding ~<<gs.RigidEntity>>~. idx: ~<0>~, uid: ~~~<<c34cfcd>>~~~, morph: ~<<gs.morphs.Plane>>~, material: ~<<gs.materials.Rigid>>~.
INFO     genesis:logger.py:127 Adding ~<<gs.RigidEntity>>~. idx: ~<1>~, uid: ~~~<<4d62d3c>>~~~, morph: ~<<gs.morphs.MJCF(file='/home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/assets/empty_scene.xml')>>~, material: ~<<gs.materials.Rigid>>~.
INFO     genesis:logger.py:127 Collision meshes are not visualized by default. To visualize them, please use `vis_mode='collision'` when calling `scene.add_entity`.
INFO     genesis:logger.py:127 Building scene ~~~<<a347606>>~~~...
INFO     genesis:logger.py:127 Compiling simulation kernels...
INFO     genesis:logger.py:127 Building visualizer...
___________________________________________ test_int_120_circuit_validation_pre_gate ___________________________________________

monkeypatch = <_pytest.monkeypatch.MonkeyPatch object at 0x71a590517f20>

    def test_int_120_circuit_validation_pre_gate(monkeypatch):
        """INT-120: Basic circuit validation failure rejects simulation."""
        # Define an electronics section (content doesn't matter as we mock validation)
        electronics = ElectronicsSection(
            power_supply=PowerSupplyConfig(voltage_dc=12, max_current_a=1),
            wiring=[],
            components=[],
        )
    
        # Mock validation to fail generically
        def mock_validate(*args):
            return CircuitValidationResult(
                valid=False,
                errors=["electronics_validation_failed: random_error"],
                node_voltages={},
            )
    
        monkeypatch.setattr("shared.pyspice_utils.validate_circuit", mock_validate)
    
        loop = SimulationLoop(
            xml_path="tests/assets/empty_scene.xml", electronics=electronics
        )
    
        # Should fail on first step due to initial validation failure check
        metrics = loop.step(control_inputs={}, duration=0.01)
    
        assert metrics.success is False
>       assert "validation_failed" in str(
            metrics.fail_reason
        ) or "electronics_validation_failed" in str(metrics.fail_reason)
E       assert ('validation_failed' in "VALIDATION_FAILED:test_int_120_circuit_validation_pre_gate.<locals>.mock_validate() got an unexpected keyword argument 'section'" or 'electronics_validation_failed' in "VALIDATION_FAILED:test_int_120_circuit_validation_pre_gate.<locals>.mock_validate() got an unexpected keyword argument 'section'")
E        +  where "VALIDATION_FAILED:test_int_120_circuit_validation_pre_gate.<locals>.mock_validate() got an unexpected keyword argument 'section'" = str("VALIDATION_FAILED:test_int_120_circuit_validation_pre_gate.<locals>.mock_validate() got an unexpected keyword argument 'section'")
E        +    where "VALIDATION_FAILED:test_int_120_circuit_validation_pre_gate.<locals>.mock_validate() got an unexpected keyword argument 'section'" = SimulationMetrics(total_time=0.0, total_energy=0.0, max_velocity=0.0, max_stress=0.0, success=False, fail_reason="VALIDATION_FAILED:test_int_120_circuit_validation_pre_gate.<locals>.mock_validate() got an unexpected keyword argument 'section'", fail_mode=<FailureReason.VALIDATION_FAILED: 'VALIDATION_FAILED'>, failure=SimulationFailure(reason=<FailureReason.VALIDATION_FAILED: 'VALIDATION_FAILED'>, detail="test_int_120_circuit_validation_pre_gate.<locals>.mock_validate() got an unexpected keyword argument 'section'"), stress_summaries=[], stress_fields={}, fluid_metrics=[], events=[], confidence='high').fail_reason
E        +  and   "VALIDATION_FAILED:test_int_120_circuit_validation_pre_gate.<locals>.mock_validate() got an unexpected keyword argument 'section'" = str("VALIDATION_FAILED:test_int_120_circuit_validation_pre_gate.<locals>.mock_validate() got an unexpected keyword argument 'section'")
E        +    where "VALIDATION_FAILED:test_int_120_circuit_validation_pre_gate.<locals>.mock_validate() got an unexpected keyword argument 'section'" = SimulationMetrics(total_time=0.0, total_energy=0.0, max_velocity=0.0, max_stress=0.0, success=False, fail_reason="VALIDATION_FAILED:test_int_120_circuit_validation_pre_gate.<locals>.mock_validate() got an unexpected keyword argument 'section'", fail_mode=<FailureReason.VALIDATION_FAILED: 'VALIDATION_FAILED'>, failure=SimulationFailure(reason=<FailureReason.VALIDATION_FAILED: 'VALIDATION_FAILED'>, detail="test_int_120_circuit_validation_pre_gate.<locals>.mock_validate() got an unexpected keyword argument 'section'"), stress_summaries=[], stress_fields={}, fluid_metrics=[], events=[], confidence='high').fail_reason

tests/electronics/test_integration_electronics.py:172: AssertionError
------------------------------------------------------ Captured log call -------------------------------------------------------
WARNING  genesis:logger.py:131 Using a simulation timestep smaller than 2ms is not recommended for 'use_gjk_collision=False' as it could lead to numerically unstable collision detection.
INFO     genesis:logger.py:127 Scene ~~~<<c7c7eda>>~~~ created.
INFO     genesis:logger.py:127 Adding ~<<gs.RigidEntity>>~. idx: ~<0>~, uid: ~~~<<e8cd1c3>>~~~, morph: ~<<gs.morphs.Plane>>~, material: ~<<gs.materials.Rigid>>~.
INFO     genesis:logger.py:127 Adding ~<<gs.RigidEntity>>~. idx: ~<1>~, uid: ~~~<<7efe68e>>~~~, morph: ~<<gs.morphs.MJCF(file='/home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/assets/empty_scene.xml')>>~, material: ~<<gs.materials.Rigid>>~.
INFO     genesis:logger.py:127 Collision meshes are not visualized by default. To visualize them, please use `vis_mode='collision'` when calling `scene.add_entity`.
INFO     genesis:logger.py:127 Building scene ~~~<<c7c7eda>>~~~...
INFO     genesis:logger.py:127 Compiling simulation kernels...
INFO     genesis:logger.py:127 Building visualizer...
_____________________________________________ test_int_121_short_circuit_detection _____________________________________________

monkeypatch = <_pytest.monkeypatch.MonkeyPatch object at 0x71a57012c4a0>

    def test_int_121_short_circuit_detection(monkeypatch):
        """INT-121: Short circuit triggers failure."""
        electronics = ElectronicsSection(
            power_supply=PowerSupplyConfig(voltage_dc=12, max_current_a=1),
            wiring=[],
            components=[],
        )
    
        # Mock validation failure
        def mock_validate(*args):
            return CircuitValidationResult(
                valid=False, errors=["FAILED_SHORT_CIRCUIT"], node_voltages={}
            )
    
        monkeypatch.setattr("shared.pyspice_utils.validate_circuit", mock_validate)
    
        loop = SimulationLoop(
            xml_path="tests/assets/empty_scene.xml", electronics=electronics
        )
    
        metrics = loop.step(control_inputs={}, duration=0.01)
        assert metrics.success is False
        # Expect failure reason to propagate
>       assert "FAILED_SHORT_CIRCUIT" in str(metrics.fail_reason)
E       assert 'FAILED_SHORT_CIRCUIT' in "VALIDATION_FAILED:test_int_121_short_circuit_detection.<locals>.mock_validate() got an unexpected keyword argument 'section'"
E        +  where "VALIDATION_FAILED:test_int_121_short_circuit_detection.<locals>.mock_validate() got an unexpected keyword argument 'section'" = str("VALIDATION_FAILED:test_int_121_short_circuit_detection.<locals>.mock_validate() got an unexpected keyword argument 'section'")
E        +    where "VALIDATION_FAILED:test_int_121_short_circuit_detection.<locals>.mock_validate() got an unexpected keyword argument 'section'" = SimulationMetrics(total_time=0.0, total_energy=0.0, max_velocity=0.0, max_stress=0.0, success=False, fail_reason="VALIDATION_FAILED:test_int_121_short_circuit_detection.<locals>.mock_validate() got an unexpected keyword argument 'section'", fail_mode=<FailureReason.VALIDATION_FAILED: 'VALIDATION_FAILED'>, failure=SimulationFailure(reason=<FailureReason.VALIDATION_FAILED: 'VALIDATION_FAILED'>, detail="test_int_121_short_circuit_detection.<locals>.mock_validate() got an unexpected keyword argument 'section'"), stress_summaries=[], stress_fields={}, fluid_metrics=[], events=[], confidence='high').fail_reason

tests/electronics/test_integration_electronics.py:200: AssertionError
------------------------------------------------------ Captured log call -------------------------------------------------------
WARNING  genesis:logger.py:131 Using a simulation timestep smaller than 2ms is not recommended for 'use_gjk_collision=False' as it could lead to numerically unstable collision detection.
INFO     genesis:logger.py:127 Scene ~~~<<83a949e>>~~~ created.
INFO     genesis:logger.py:127 Adding ~<<gs.RigidEntity>>~. idx: ~<0>~, uid: ~~~<<fb2b6ad>>~~~, morph: ~<<gs.morphs.Plane>>~, material: ~<<gs.materials.Rigid>>~.
INFO     genesis:logger.py:127 Adding ~<<gs.RigidEntity>>~. idx: ~<1>~, uid: ~~~<<325c0fe>>~~~, morph: ~<<gs.morphs.MJCF(file='/home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/assets/empty_scene.xml')>>~, material: ~<<gs.materials.Rigid>>~.
INFO     genesis:logger.py:127 Collision meshes are not visualized by default. To visualize them, please use `vis_mode='collision'` when calling `scene.add_entity`.
INFO     genesis:logger.py:127 Building scene ~~~<<83a949e>>~~~...
INFO     genesis:logger.py:127 Compiling simulation kernels...
INFO     genesis:logger.py:127 Building visualizer...
__________________________________________ test_int_122_overcurrent_supply_detection ___________________________________________

monkeypatch = <_pytest.monkeypatch.MonkeyPatch object at 0x71a518554950>

    def test_int_122_overcurrent_supply_detection(monkeypatch):
        """INT-122: Overcurrent supply triggers failure."""
        electronics = ElectronicsSection(
            power_supply=PowerSupplyConfig(voltage_dc=12, max_current_a=1),
            wiring=[],
            components=[],
        )
    
        def mock_validate(*args):
            return CircuitValidationResult(
                valid=False,
                errors=["FAILED_OVERCURRENT_SUPPLY"],
                node_voltages={},
                total_draw_a=100.0,
            )
    
        monkeypatch.setattr("shared.pyspice_utils.validate_circuit", mock_validate)
    
        loop = SimulationLoop(
            xml_path="tests/assets/empty_scene.xml", electronics=electronics
        )
    
        metrics = loop.step(control_inputs={}, duration=0.01)
        assert metrics.success is False
>       assert "FAILED_OVERCURRENT_SUPPLY" in str(metrics.fail_reason)
E       assert 'FAILED_OVERCURRENT_SUPPLY' in "VALIDATION_FAILED:test_int_122_overcurrent_supply_detection.<locals>.mock_validate() got an unexpected keyword argument 'section'"
E        +  where "VALIDATION_FAILED:test_int_122_overcurrent_supply_detection.<locals>.mock_validate() got an unexpected keyword argument 'section'" = str("VALIDATION_FAILED:test_int_122_overcurrent_supply_detection.<locals>.mock_validate() got an unexpected keyword argument 'section'")
E        +    where "VALIDATION_FAILED:test_int_122_overcurrent_supply_detection.<locals>.mock_validate() got an unexpected keyword argument 'section'" = SimulationMetrics(total_time=0.0, total_energy=0.0, max_velocity=0.0, max_stress=0.0, success=False, fail_reason="VALIDATION_FAILED:test_int_122_overcurrent_supply_detection.<locals>.mock_validate() got an unexpected keyword argument 'section'", fail_mode=<FailureReason.VALIDATION_FAILED: 'VALIDATION_FAILED'>, failure=SimulationFailure(reason=<FailureReason.VALIDATION_FAILED: 'VALIDATION_FAILED'>, detail="test_int_122_overcurrent_supply_detection.<locals>.mock_validate() got an unexpected keyword argument 'section'"), stress_summaries=[], stress_fields={}, fluid_metrics=[], events=[], confidence='high').fail_reason

tests/electronics/test_integration_electronics.py:227: AssertionError
------------------------------------------------------ Captured log call -------------------------------------------------------
WARNING  genesis:logger.py:131 Using a simulation timestep smaller than 2ms is not recommended for 'use_gjk_collision=False' as it could lead to numerically unstable collision detection.
INFO     genesis:logger.py:127 Scene ~~~<<c01a10c>>~~~ created.
INFO     genesis:logger.py:127 Adding ~<<gs.RigidEntity>>~. idx: ~<0>~, uid: ~~~<<d1700b8>>~~~, morph: ~<<gs.morphs.Plane>>~, material: ~<<gs.materials.Rigid>>~.
INFO     genesis:logger.py:127 Adding ~<<gs.RigidEntity>>~. idx: ~<1>~, uid: ~~~<<6d56979>>~~~, morph: ~<<gs.morphs.MJCF(file='/home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/assets/empty_scene.xml')>>~, material: ~<<gs.materials.Rigid>>~.
INFO     genesis:logger.py:127 Collision meshes are not visualized by default. To visualize them, please use `vis_mode='collision'` when calling `scene.add_entity`.
INFO     genesis:logger.py:127 Building scene ~~~<<c01a10c>>~~~...
INFO     genesis:logger.py:127 Compiling simulation kernels...
INFO     genesis:logger.py:127 Building visualizer...
_____________________________________________ test_int_124_open_circuit_detection ______________________________________________

monkeypatch = <_pytest.monkeypatch.MonkeyPatch object at 0x71a5185f54c0>

    def test_int_124_open_circuit_detection(monkeypatch):
        """INT-124: Open circuit/floating node triggers failure."""
        electronics = ElectronicsSection(
            power_supply=PowerSupplyConfig(voltage_dc=12, max_current_a=1),
            wiring=[],
            components=[],
        )
    
        def mock_validate(*args):
            return CircuitValidationResult(
                valid=False, errors=["FAILED_OPEN_CIRCUIT:node_x"], node_voltages={}
            )
    
        monkeypatch.setattr("shared.pyspice_utils.validate_circuit", mock_validate)
    
        loop = SimulationLoop(
            xml_path="tests/assets/empty_scene.xml", electronics=electronics
        )
    
        metrics = loop.step(control_inputs={}, duration=0.01)
        assert metrics.success is False
>       assert "FAILED_OPEN_CIRCUIT" in str(metrics.fail_reason)
E       assert 'FAILED_OPEN_CIRCUIT' in "VALIDATION_FAILED:test_int_124_open_circuit_detection.<locals>.mock_validate() got an unexpected keyword argument 'section'"
E        +  where "VALIDATION_FAILED:test_int_124_open_circuit_detection.<locals>.mock_validate() got an unexpected keyword argument 'section'" = str("VALIDATION_FAILED:test_int_124_open_circuit_detection.<locals>.mock_validate() got an unexpected keyword argument 'section'")
E        +    where "VALIDATION_FAILED:test_int_124_open_circuit_detection.<locals>.mock_validate() got an unexpected keyword argument 'section'" = SimulationMetrics(total_time=0.0, total_energy=0.0, max_velocity=0.0, max_stress=0.0, success=False, fail_reason="VALIDATION_FAILED:test_int_124_open_circuit_detection.<locals>.mock_validate() got an unexpected keyword argument 'section'", fail_mode=<FailureReason.VALIDATION_FAILED: 'VALIDATION_FAILED'>, failure=SimulationFailure(reason=<FailureReason.VALIDATION_FAILED: 'VALIDATION_FAILED'>, detail="test_int_124_open_circuit_detection.<locals>.mock_validate() got an unexpected keyword argument 'section'"), stress_summaries=[], stress_fields={}, fluid_metrics=[], events=[], confidence='high').fail_reason

tests/electronics/test_integration_electronics.py:251: AssertionError
------------------------------------------------------ Captured log call -------------------------------------------------------
WARNING  genesis:logger.py:131 Using a simulation timestep smaller than 2ms is not recommended for 'use_gjk_collision=False' as it could lead to numerically unstable collision detection.
INFO     genesis:logger.py:127 Scene ~~~<<1d36a3c>>~~~ created.
INFO     genesis:logger.py:127 Adding ~<<gs.RigidEntity>>~. idx: ~<0>~, uid: ~~~<<4d48b0d>>~~~, morph: ~<<gs.morphs.Plane>>~, material: ~<<gs.materials.Rigid>>~.
INFO     genesis:logger.py:127 Adding ~<<gs.RigidEntity>>~. idx: ~<1>~, uid: ~~~<<8bd942f>>~~~, morph: ~<<gs.morphs.MJCF(file='/home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/assets/empty_scene.xml')>>~, material: ~<<gs.materials.Rigid>>~.
INFO     genesis:logger.py:127 Collision meshes are not visualized by default. To visualize them, please use `vis_mode='collision'` when calling `scene.add_entity`.
INFO     genesis:logger.py:127 Building scene ~~~<<1d36a3c>>~~~...
INFO     genesis:logger.py:127 Compiling simulation kernels...
INFO     genesis:logger.py:127 Building visualizer...
___________________________________________________ test_coder_node_success ____________________________________________________

self = <Coroutine test_coder_node_success>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a50826cda0>, coro = <coroutine object test_coder_node_success at 0x71a50828b920>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
________________________________________________ test_coder_node_with_feedback _________________________________________________

self = <Coroutine test_coder_node_with_feedback>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a50824b710>
coro = <coroutine object test_coder_node_with_feedback at 0x71a5908adfe0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
___________________________________________ test_coder_node_injects_objectives_yaml ____________________________________________

self = <Coroutine test_coder_node_injects_objectives_yaml>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a50826c770>
coro = <coroutine object test_coder_node_injects_objectives_yaml at 0x71a590abb400>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
__________________________________________________ test_define_graph_compiles __________________________________________________

self = <Coroutine test_define_graph_compiles>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a508248a70>, coro = <coroutine object test_define_graph_compiles at 0x71a51814c940>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
______________________________________________ test_run_generation_session_mocked ______________________________________________

self = <Coroutine test_run_generation_session_mocked>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a508248f80>
coro = <coroutine object test_run_generation_session_mocked at 0x71a50828e200>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
_____________________________________________ test_run_generation_session_rejected _____________________________________________

self = <Coroutine test_run_generation_session_rejected>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a508271790>
coro = <coroutine object test_run_generation_session_rejected at 0x71a590abb9c0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
____________________________________________ test_planner_node_prompt_construction _____________________________________________

self = <Coroutine test_planner_node_prompt_construction>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5082704a0>
coro = <coroutine object test_planner_node_prompt_construction at 0x71a6e5f26260>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
_____________________________________________ test_coder_node_prompt_construction ______________________________________________

self = <Coroutine test_coder_node_prompt_construction>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a508324320>
coro = <coroutine object test_coder_node_prompt_construction at 0x71a590abbe10>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
__________________________________________ test_int_001_compose_boot_health_contract ___________________________________________

self = <Coroutine test_int_001_compose_boot_health_contract>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a508270530>
coro = <coroutine object test_int_001_compose_boot_health_contract at 0x71a5907b5840>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
______________________________________ test_int_002_controller_worker_execution_boundary _______________________________________

self = <Coroutine test_int_002_controller_worker_execution_boundary>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5083277a0>
coro = <coroutine object test_int_002_controller_worker_execution_boundary at 0x71a5908af320>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
__________________________________________ test_int_003_session_filesystem_isolation ___________________________________________

self = <Coroutine test_int_003_session_filesystem_isolation>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5082e6210>
coro = <coroutine object test_int_003_session_filesystem_isolation at 0x71a590670e80>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
____________________________________________ test_int_004_simulation_serialization _____________________________________________

self = <Coroutine test_int_004_simulation_serialization>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5082e6300>
coro = <coroutine object test_int_004_simulation_serialization at 0x71a590add8a0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
___________________________________________ test_int_020_simulation_failure_taxonomy ___________________________________________

self = <Coroutine test_int_020_simulation_failure_taxonomy>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5082e79e0>
coro = <coroutine object test_int_020_simulation_failure_taxonomy at 0x71a5185b36f0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
________________________________________ test_int_021_runtime_randomization_robustness _________________________________________

self = <Coroutine test_int_021_runtime_randomization_robustness>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5082e8d40>
coro = <coroutine object test_int_021_runtime_randomization_robustness at 0x71a5907b5fc0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
_____________________________________________ test_int_022_motor_overload_behavior _____________________________________________

self = <Coroutine test_int_022_motor_overload_behavior>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5082e61e0>
coro = <coroutine object test_int_022_motor_overload_behavior at 0x71a590abb6e0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
_____________________________________________ test_int_023_fastener_validity_rules _____________________________________________

self = <Coroutine test_int_023_fastener_validity_rules>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5082e4f20>
coro = <coroutine object test_int_023_fastener_validity_rules at 0x71a6e5f26260>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
______________________________________ test_int_012_013_cots_search_contract_and_readonly ______________________________________

self = <Coroutine test_int_012_013_cots_search_contract_and_readonly>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5082e9a00>
coro = <coroutine object test_int_012_013_cots_search_contract_and_readonly at 0x4ad81200>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
__________________________________________ test_int_016_reviewer_decision_schema_gate __________________________________________

self = <Coroutine test_int_016_reviewer_decision_schema_gate>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5082ea5a0>
coro = <coroutine object test_int_016_reviewer_decision_schema_gate at 0x71a5907b59c0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
________________________________________________ test_int_017_plan_refusal_loop ________________________________________________

self = <Coroutine test_int_017_plan_refusal_loop>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a508327860>
coro = <coroutine object test_int_017_plan_refusal_loop at 0x71a5907b6140>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
____________________________________________ test_int_026_mandatory_event_families _____________________________________________

self = <Coroutine test_int_026_mandatory_event_families>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5b09dddf0>
coro = <coroutine object test_int_026_mandatory_event_families at 0x71a50831c1b0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
______________________________________________ test_int_027_seed_variant_tracking ______________________________________________

self = <Coroutine test_int_027_seed_variant_tracking>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5082eb080>
coro = <coroutine object test_int_027_seed_variant_tracking at 0x71a590925a80>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
___________________________________________ test_int_028_strict_api_schema_contract ____________________________________________

self = <Coroutine test_int_028_strict_api_schema_contract>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5b205a330>
coro = <coroutine object test_int_028_strict_api_schema_contract at 0x71a50828bac0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
_______________________________________________ test_int_029_api_key_enforcement _______________________________________________

self = <Coroutine test_int_029_api_key_enforcement>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5082d89e0>
coro = <coroutine object test_int_029_api_key_enforcement at 0x71a590925380>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
______________________________________________ test_int_030_interrupt_propagation ______________________________________________

self = <Coroutine test_int_030_interrupt_propagation>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5082d8110>
coro = <coroutine object test_int_030_interrupt_propagation at 0x71a590abb9c0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
_____________________________________________ test_int_061_asset_serving_security ______________________________________________

self = <Coroutine test_int_061_asset_serving_security>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5082d9a00>
coro = <coroutine object test_int_061_asset_serving_security at 0x71a6e5f26260>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
_____________________________________________ test_int_062_worker_openapi_contract _____________________________________________

self = <Coroutine test_int_062_worker_openapi_contract>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a57012d6d0>
coro = <coroutine object test_int_062_worker_openapi_contract at 0x71a59097dfc0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
_____________________________________________ test_int_063_mounted_path_read_only ______________________________________________

self = <Coroutine test_int_063_mounted_path_read_only>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5082e45c0>
coro = <coroutine object test_int_063_mounted_path_read_only at 0x71a5906710e0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
___________________________________________ test_int_102_111_fem_material_validation ___________________________________________

self = <Coroutine test_int_102_111_fem_material_validation>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5082eb320>
coro = <coroutine object test_int_102_111_fem_material_validation at 0x71a590925380>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
_____________________________________________ test_int_103_part_breakage_detection _____________________________________________

self = <Coroutine test_int_103_part_breakage_detection>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5082eb0b0>
coro = <coroutine object test_int_103_part_breakage_detection at 0x71a50822dad0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
________________________________________________ test_int_104_stress_reporting _________________________________________________

self = <Coroutine test_int_104_stress_reporting>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5082a8fb0>
coro = <coroutine object test_int_104_stress_reporting at 0x71a590925a80>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
___________________________________________ test_int_107_stress_objective_evaluation ___________________________________________

self = <Coroutine test_int_107_stress_objective_evaluation>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5082aa960>
coro = <coroutine object test_int_107_stress_objective_evaluation at 0x71a5907b5cc0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
____________________________________________ test_int_109_physics_instability_abort ____________________________________________

self = <Coroutine test_int_109_physics_instability_abort>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5082aa0f0>
coro = <coroutine object test_int_109_physics_instability_abort at 0x71a6e5f26260>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
___________________________________________ test_int_108_tetrahedralization_pipeline ___________________________________________

self = <Coroutine test_int_108_tetrahedralization_pipeline>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5083272c0>
coro = <coroutine object test_int_108_tetrahedralization_pipeline at 0x71a590925a80>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
__________________________________________________ test_int_110_gpu_oom_retry __________________________________________________

self = <Coroutine test_int_110_gpu_oom_retry>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5082e7ec0>, coro = <coroutine object test_int_110_gpu_oom_retry at 0x71a5905c96c0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
_____________________________________________ test_int_120_circuit_validation_gate _____________________________________________

self = <Coroutine test_int_120_circuit_validation_gate>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5082726f0>
coro = <coroutine object test_int_120_circuit_validation_gate at 0x71a5907b56c0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
_____________________________________________ test_int_121_short_circuit_detection _____________________________________________

self = <Coroutine test_int_121_short_circuit_detection>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5082e1520>
coro = <coroutine object test_int_121_short_circuit_detection at 0x71a59097de70>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
__________________________________________ test_int_122_overcurrent_supply_detection ___________________________________________

self = <Coroutine test_int_122_overcurrent_supply_detection>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5082e2cf0>
coro = <coroutine object test_int_122_overcurrent_supply_detection at 0x71a5908afe20>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
___________________________________________ test_int_123_overcurrent_wire_detection ____________________________________________

self = <Coroutine test_int_123_overcurrent_wire_detection>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5082abd70>
coro = <coroutine object test_int_123_overcurrent_wire_detection at 0x71a5082f4720>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
_____________________________________________ test_int_124_open_circuit_detection ______________________________________________

self = <Coroutine test_int_124_open_circuit_detection>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5082e25d0>
coro = <coroutine object test_int_124_open_circuit_detection at 0x71a59097e110>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
__________________________________________ test_int_004_episode_artifact_persistence ___________________________________________

self = <Coroutine test_int_004_episode_artifact_persistence>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5082e3b30>
coro = <coroutine object test_int_004_episode_artifact_persistence at 0x71a5905c8400>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
____________________________________________ test_int_005_trace_realtime_broadcast _____________________________________________

self = <Coroutine test_int_005_trace_realtime_broadcast>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5082e1970>
coro = <coroutine object test_int_005_trace_realtime_broadcast at 0x71a5908aec40>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
_________________________________________ test_int_011_planner_target_caps_validation __________________________________________

self = <Coroutine test_int_011_planner_target_caps_validation>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5082eb3b0>
coro = <coroutine object test_int_011_planner_target_caps_validation at 0x71a5185a63e0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
________________________________________________ test_int_014_cots_propagation _________________________________________________

self = <Coroutine test_int_014_cots_propagation>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5082e7fe0>
coro = <coroutine object test_int_014_cots_propagation at 0x71a5907b5840>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
______________________________________________ test_int_025_events_collection_e2e ______________________________________________

self = <Coroutine test_int_025_events_collection_e2e>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a4fc494cb0>
coro = <coroutine object test_int_025_events_collection_e2e at 0x71a5082f49e0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
___________________________________________ test_int_053_temporal_workflow_lifecycle ___________________________________________

self = <Coroutine test_int_053_temporal_workflow_lifecycle>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5082e1d00>
coro = <coroutine object test_int_053_temporal_workflow_lifecycle at 0x71a590abbca0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
___________________________________________ test_int_055_s3_artifact_upload_logging ____________________________________________

self = <Coroutine test_int_055_s3_artifact_upload_logging>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5082e39e0>
coro = <coroutine object test_int_055_s3_artifact_upload_logging at 0x71a5907b56c0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
______________________________________________ test_int_054_temporal_failure_path ______________________________________________

self = <Coroutine test_int_054_temporal_failure_path>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a50824b740>
coro = <coroutine object test_int_054_temporal_failure_path at 0x71a5908add20>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
_____________________________________________ test_int_056_s3_upload_failure_retry _____________________________________________

self = <Coroutine test_int_056_s3_upload_failure_retry>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a4fc494830>
coro = <coroutine object test_int_056_s3_upload_failure_retry at 0x71a5905c8400>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
____________________________________________ test_int_101_physics_backend_selection ____________________________________________

self = <Coroutine test_int_101_physics_backend_selection>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a4fc496b40>
coro = <coroutine object test_int_101_physics_backend_selection at 0x71a5907b68c0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
__________________________________________ test_int_105_fluid_containment_evaluation ___________________________________________

self = <Coroutine test_int_105_fluid_containment_evaluation>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5082e4ef0>
coro = <coroutine object test_int_105_fluid_containment_evaluation at 0x71a50828bac0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
______________________________________________ test_int_106_flow_rate_evaluation _______________________________________________

self = <Coroutine test_int_106_flow_rate_evaluation>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5082ea570>
coro = <coroutine object test_int_106_flow_rate_evaluation at 0x71a590670e80>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
_____________________________________________ test_int_112_mujoco_backward_compat ______________________________________________

self = <Coroutine test_int_112_mujoco_backward_compat>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5082db0b0>
coro = <coroutine object test_int_112_mujoco_backward_compat at 0x71a5908ae560>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
____________________________________________ test_int_005_mandatory_artifacts_gate _____________________________________________

self = <Coroutine test_int_005_mandatory_artifacts_gate>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5082e4aa0>
coro = <coroutine object test_int_005_mandatory_artifacts_gate at 0x71a590abb9c0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
____________________________________________ test_int_006_plan_structure_validation ____________________________________________

self = <Coroutine test_int_006_plan_structure_validation>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a4fc46d700>
coro = <coroutine object test_int_006_plan_structure_validation at 0x71a5082f5900>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
_________________________________________________ test_int_007_todo_integrity __________________________________________________

self = <Coroutine test_int_007_todo_integrity>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a4fc46ca70>
coro = <coroutine object test_int_007_todo_integrity at 0x71a5082f5380>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
______________________________________________ test_int_008_objectives_validation ______________________________________________

self = <Coroutine test_int_008_objectives_validation>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a4fc46c1a0>
coro = <coroutine object test_int_008_objectives_validation at 0x71a5082f5a60>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
___________________________________________ test_int_009_cost_estimation_validation ____________________________________________

self = <Coroutine test_int_009_cost_estimation_validation>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a508273e30>
coro = <coroutine object test_int_009_cost_estimation_validation at 0x71a5908ade80>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
____________________________________________ test_int_011_planner_caps_enforcement _____________________________________________

self = <Coroutine test_int_011_planner_caps_enforcement>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a4fc46cb90>
coro = <coroutine object test_int_011_planner_caps_enforcement at 0x71a5908aec40>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
_________________________________________ test_int_015_engineer_handover_immutability __________________________________________

self = <Coroutine test_int_015_engineer_handover_immutability>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a4fc46df10>
coro = <coroutine object test_int_015_engineer_handover_immutability at 0x71a590abb9c0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
_____________________________________________ test_int_019_hard_constraints_gates ______________________________________________

self = <Coroutine test_int_019_hard_constraints_gates>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a4fc46dbe0>
coro = <coroutine object test_int_019_hard_constraints_gates at 0x413d0750>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
_______________________________________ test_int_010_planner_pricing_script_integration ________________________________________

self = <Coroutine test_int_010_planner_pricing_script_integration>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a4fc46c890>
coro = <coroutine object test_int_010_planner_pricing_script_integration at 0x71a5907b5cc0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
_______________________________________ test_int_018_validate_and_price_integration_gate _______________________________________

self = <Coroutine test_int_018_validate_and_price_integration_gate>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a4fc4ecaa0>
coro = <coroutine object test_int_018_validate_and_price_integration_gate at 0x71a50831cd30>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
_____________________________________________ test_api_fuzzing[GET /cots/metadata] _____________________________________________
+ Exception Group Traceback (most recent call last):
  |   File "/home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/integration/architecture_p1/test_api_fuzzing.py", line 52, in test_api_fuzzing
  |     case.call_and_validate(checks=(schemathesis.checks.not_a_server_error,))
  |   File "/home/maksym/Work/proj/Problemologist/Problemologist-AI/.venv/lib/python3.12/site-packages/schemathesis/generation/case.py", line 512, in call_and_validate
  |     self.validate_response(
  |   File "/home/maksym/Work/proj/Problemologist/Problemologist-AI/.venv/lib/python3.12/site-packages/schemathesis/generation/case.py", line 478, in validate_response
  |     raise FailureGroup(_failures, message) from None
  | schemathesis.core.failures.FailureGroup: Schemathesis found 1 distinct failure
  | 
  | - Server error
  | 
  | [500] Internal Server Error:
  | 
  |     `Internal Server Error`
  | 
  | Reproduce with:
  | 
  |     curl -X GET -H 'X-Session-ID: [Filtered]' http://localhost:18000/cots/metadata
  | 
  |  (1 sub-exception)
  +-+---------------- 1 ----------------
    | schemathesis.core.failures.ServerError: Server error
    +------------------------------------
______________________________________________ test_int_043_batch_execution_path _______________________________________________

self = <Coroutine test_int_043_batch_execution_path>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a529178530>
coro = <coroutine object test_int_043_batch_execution_path at 0x71a4fc173490>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
___________________________________________ test_benchmark_planner_cad_reviewer_path ___________________________________________

self = <Coroutine test_benchmark_planner_cad_reviewer_path>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a529049df0>
coro = <coroutine object test_benchmark_planner_cad_reviewer_path at 0x71a529055ad0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
__________________________________________________ test_engineering_full_loop __________________________________________________

self = <Coroutine test_engineering_full_loop>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a52904a6c0>, coro = <coroutine object test_engineering_full_loop at 0x71a590adf100>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
______________________________________________ test_benchmark_to_engineer_handoff ______________________________________________

self = <Coroutine test_benchmark_to_engineer_handoff>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a529071970>
coro = <coroutine object test_benchmark_to_engineer_handoff at 0x71a50828f130>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
___________________________________________ test_render_artifact_generation_int_039 ____________________________________________

self = <Coroutine test_render_artifact_generation_int_039>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5290482c0>
coro = <coroutine object test_render_artifact_generation_int_039 at 0x71a57090be00>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
____________________________________________ test_asset_persistence_linkage_int_040 ____________________________________________

self = <Coroutine test_asset_persistence_linkage_int_040>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a529071ac0>
coro = <coroutine object test_asset_persistence_linkage_int_040 at 0x71a52913c040>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
_______________________________________________ test_mjcf_joint_mapping_int_037 ________________________________________________

self = <Coroutine test_mjcf_joint_mapping_int_037>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a529070f20>
coro = <coroutine object test_mjcf_joint_mapping_int_037 at 0x71a6e5f26650>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
___________________________________________ test_controller_function_family_int_038 ____________________________________________

self = <Coroutine test_controller_function_family_int_038>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a529073950>
coro = <coroutine object test_controller_function_family_int_038 at 0x71a4fc2bb0d0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
________________________________________________ test_temporal_recovery_int_041 ________________________________________________

self = <Coroutine test_temporal_recovery_int_041>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5290b0aa0>
coro = <coroutine object test_temporal_recovery_int_041 at 0x71a6e5dccfb0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
_________________________________________________ test_async_callbacks_int_042 _________________________________________________

self = <Coroutine test_async_callbacks_int_042>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5290b00b0>
coro = <coroutine object test_async_callbacks_int_042 at 0x71a529aba980>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
__________________________________________________ test_int_064_cots_metadata __________________________________________________

self = <Coroutine test_int_064_cots_metadata>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a52904b560>, coro = <coroutine object test_int_064_cots_metadata at 0x71a5292ad9c0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
__________________________________________________ test_int_065_skills_safety __________________________________________________

self = <Coroutine test_int_065_skills_safety>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5290b01a0>, coro = <coroutine object test_int_065_skills_safety at 0x71a570109cf0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
___________________________________________ test_int_066_fluid_electronics_coupling ____________________________________________

self = <Coroutine test_int_066_fluid_electronics_coupling>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5290b3590>
coro = <coroutine object test_int_066_fluid_electronics_coupling at 0x71a57090be00>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
________________________________________________ test_int_067_068_steerability _________________________________________________

self = <Coroutine test_int_067_068_steerability>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5291e6420>
coro = <coroutine object test_int_067_068_steerability at 0x71a5292ac640>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
________________________________________________ test_int_069_frontend_contract ________________________________________________

self = <Coroutine test_int_069_frontend_contract>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5290cc3e0>
coro = <coroutine object test_int_069_frontend_contract at 0x71a50831db90>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
___________________________________________ test_manufacturing_methods_and_materials ___________________________________________

self = <Coroutine test_manufacturing_methods_and_materials>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5290cc170>
coro = <coroutine object test_manufacturing_methods_and_materials at 0x71a590adf100>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
_____________________________________________ test_int_059_langfuse_trace_linkage ______________________________________________

self = <Coroutine test_int_059_langfuse_trace_linkage>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a529072180>
coro = <coroutine object test_int_059_langfuse_trace_linkage at 0x71a50828f2e0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
___________________________________________ test_int_060_langfuse_feedback_contract ____________________________________________

self = <Coroutine test_int_060_langfuse_feedback_contract>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5290b09b0>
coro = <coroutine object test_int_060_langfuse_feedback_contract at 0x71a529d6a880>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
_______________________________________________ test_int_057_backup_logging_flow _______________________________________________

self = <Coroutine test_int_057_backup_logging_flow>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5290cc710>
coro = <coroutine object test_int_057_backup_logging_flow at 0x71a5292ac640>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
____________________________________________ test_int_058_cross_system_correlation _____________________________________________

self = <Coroutine test_int_058_cross_system_correlation>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5290b2a80>
coro = <coroutine object test_int_058_cross_system_correlation at 0x71a529c29120>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
_________________________________________________ test_int_138_smoke_test_mode _________________________________________________

self = <Coroutine test_int_138_smoke_test_mode>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a529048d10>
coro = <coroutine object test_int_138_smoke_test_mode at 0x71a52913ccc0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
______________________________________________ test_int_139_fluid_storage_policy _______________________________________________

self = <Coroutine test_int_139_fluid_storage_policy>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5290984a0>
coro = <coroutine object test_int_139_fluid_storage_policy at 0x71a5292ad0c0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
_____________________________________________ test_reviewer_evidence_completeness ______________________________________________

self = <Coroutine test_reviewer_evidence_completeness>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5290b32c0>
coro = <coroutine object test_reviewer_evidence_completeness at 0x71a529c29120>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
______________________________________________ test_int_045_skills_sync_lifecycle ______________________________________________

self = <Coroutine test_int_045_skills_sync_lifecycle>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5291cd2b0>
coro = <coroutine object test_int_045_skills_sync_lifecycle at 0x71a50828f2e0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
_________________________________________ test_plan_to_cad_fidelity_regression_int_046 _________________________________________

self = <Coroutine test_plan_to_cad_fidelity_regression_int_046>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5291cdd60>
coro = <coroutine object test_plan_to_cad_fidelity_regression_int_046 at 0x71a5b1f0da20>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
___________________________________________ test_cross_seed_transfer_uplift_int_047 ____________________________________________

self = <Coroutine test_cross_seed_transfer_uplift_int_047>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a529371ee0>
coro = <coroutine object test_cross_seed_transfer_uplift_int_047 at 0x71a5185b22a0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
_________________________________________ test_reviewer_optimality_regression_int_048 __________________________________________

self = <Coroutine test_reviewer_optimality_regression_int_048>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5293737a0>
coro = <coroutine object test_reviewer_optimality_regression_int_048 at 0x71a5908ad4e0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
________________________________________ test_evaluation_metric_materialization_int_049 ________________________________________

self = <Coroutine test_evaluation_metric_materialization_int_049>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5293ba7e0>
coro = <coroutine object test_evaluation_metric_materialization_int_049 at 0x71a5b196c190>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
_________________________________________ test_dataset_readiness_completeness_int_050 __________________________________________

self = <Coroutine test_dataset_readiness_completeness_int_050>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5293ba570>
coro = <coroutine object test_dataset_readiness_completeness_int_050 at 0x71a4e44cb920>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
___________________________________________ test_journal_quality_integration_int_051 ___________________________________________

self = <Coroutine test_journal_quality_integration_int_051>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a52917a2d0>
coro = <coroutine object test_journal_quality_integration_int_051 at 0x71a50828f130>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
__________________________________________ test_skill_effectiveness_tracking_int_052 ___________________________________________

self = <Coroutine test_skill_effectiveness_tracking_int_052>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5290b1e50>
coro = <coroutine object test_skill_effectiveness_tracking_int_052 at 0x71a5b1f1fd00>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
________________________________________________ test_electromechanical_handoff ________________________________________________

self = <Coroutine test_electromechanical_handoff>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a52933f710>
coro = <coroutine object test_electromechanical_handoff at 0x71a5908adbc0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
____________________________________________ test_electronics_engineer_passthrough _____________________________________________

self = <Coroutine test_electronics_engineer_passthrough>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a52933c710>
coro = <coroutine object test_electronics_engineer_passthrough at 0x71a5181f6240>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
_________________________________________________ test_fem_breakage_detection __________________________________________________

tmp_path = PosixPath('/tmp/pytest-of-maksym/pytest-2/test_fem_breakage_detection0')

    @pytest.mark.integration
    def test_fem_breakage_detection(tmp_path):
        """
        Verify that exceeding ultimate stress triggers PART_BREAKAGE failure.
        We mock the stress field to simulate high load.
        """
        xml_path = tmp_path / "scene.xml"
        xml_content = """
    <mujoco model="fem_test">
        <worldbody>
            <body name="deformable_part" pos="0 0 1">
                <geom name="part_geom" type="box" size="0.1 0.1 0.1"/>
            </body>
        </worldbody>
    </mujoco>
    """
        xml_path.write_text(xml_content)
    
        with patch("worker_heavy.simulation.loop.get_physics_backend") as mock_get_backend:
            mock_backend = MagicMock()
            mock_get_backend.return_value = mock_backend
    
            # Setup mock backend behavior
            mock_backend.get_all_body_names.return_value = ["deformable_part"]
            mock_backend.step.return_value = MagicMock(time=0.002, success=True)
    
            # High stress exceeding default ultimate (310MPa)
            high_stress = 400e6
            mock_backend.get_stress_field.return_value = StressField(
                nodes=np.array([[0, 0, 0]]), stress=np.array([high_stress])
            )
    
            loop = SimulationLoop(str(xml_path), backend_type=SimulatorBackendType.GENESIS)
    
            # We need to ensure smoke_test_mode is True or step once
            loop.smoke_test_mode = True
    
>           metrics = loop.step(control_inputs={}, duration=0.1)
                      ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

tests/integration/test_fem_breakage.py:49: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
worker_heavy/simulation/loop.py:388: in step
    if self._check_simulation_failure(
worker_heavy/simulation/loop.py:568: in _check_simulation_failure
    self.metric_collector.update(dt_interval, energy, target_vel, max_stress)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <worker_heavy.simulation.metrics.MetricCollector object at 0x71a529179370>
delta_time = <MagicMock name='get_physics_backend().timestep.__mul__()' id='124954172577200'>, energy = 0, velocity = 0.0
stress = <MagicMock name='get_physics_backend().get_max_stress()' id='124954175022016'>

    def update(self, delta_time: float, energy: float, velocity: float, stress: float):
        """Update metrics with new values."""
        self.metrics.total_time += delta_time
        self.metrics.total_energy += energy
        self.metrics.max_velocity = max(self.metrics.max_velocity, velocity)
>       self.metrics.max_stress = max(self.metrics.max_stress, stress)
                                  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
E       TypeError: '>' not supported between instances of 'MagicMock' and 'float'

worker_heavy/simulation/metrics.py:19: TypeError
_________________________________________________ test_fluid_containment_logic _________________________________________________

tmp_path = PosixPath('/tmp/pytest-of-maksym/pytest-2/test_fluid_containment_logic0')

    @pytest.mark.integration
    def test_fluid_containment_logic(tmp_path):
        """
        Verify that fluid particles are correctly counted against zones in SimulationLoop.
        """
        xml_path = tmp_path / "scene.xml"
        xml_path.write_text("<mujoco/>")
    
        objectives_dict = {
            "objectives": {
                "goal_zone": {"min": [10, 10, 10], "max": [12, 12, 12]},
                "build_zone": {"min": [-5, -5, -5], "max": [5, 5, 5]},
                "fluid_objectives": [
                    {
                        "type": "fluid_containment",
                        "fluid_id": "water",
                        "containment_zone": {"min": [-1, -1, -1], "max": [1, 1, 1]},
                        "threshold": 0.9,
                        "eval_at": "end",
                    }
                ],
            },
            "simulation_bounds": {"min": [-20, -20, -20], "max": [20, 20, 20]},
            "moved_object": {
                "label": "obj",
                "shape": "sphere",
                "start_position": [0, 0, 0],
                "runtime_jitter": [0, 0, 0],
            },
            "constraints": {"max_unit_cost": 100, "max_weight_g": 10},
        }
    
        with patch("worker_heavy.simulation.loop.get_physics_backend") as mock_get_backend:
            mock_backend = MagicMock()
            mock_get_backend.return_value = mock_backend
            mock_backend.step.return_value = MagicMock(time=0.1, success=True)
    
            # Mock particles: 100 particles, 95 inside the zone
            particles = np.zeros((100, 3))
            # 95 at (0,0,0) - inside [-1, 1]
            # 5 at (5,5,5) - outside
            particles[95:] = [5, 5, 5]
            mock_backend.get_particle_positions.return_value = particles
    
            loop = SimulationLoop(
                str(xml_path),
                backend_type=SimulatorBackendType.GENESIS,
                objectives=ObjectivesYaml(**objectives_dict),
            )
    
>           metrics = loop.step(control_inputs={}, duration=0.1)
                      ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

tests/integration/test_fluid_containment.py:61: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
worker_heavy/simulation/loop.py:388: in step
    if self._check_simulation_failure(
worker_heavy/simulation/loop.py:568: in _check_simulation_failure
    self.metric_collector.update(dt_interval, energy, target_vel, max_stress)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <worker_heavy.simulation.metrics.MetricCollector object at 0x71a529504800>
delta_time = <MagicMock name='get_physics_backend().timestep.__mul__()' id='124954171711008'>, energy = 0, velocity = 0.0
stress = <MagicMock name='get_physics_backend().get_max_stress()' id='124954178766656'>

    def update(self, delta_time: float, energy: float, velocity: float, stress: float):
        """Update metrics with new values."""
        self.metrics.total_time += delta_time
        self.metrics.total_energy += energy
        self.metrics.max_velocity = max(self.metrics.max_velocity, velocity)
>       self.metrics.max_stress = max(self.metrics.max_stress, stress)
                                  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
E       TypeError: '>' not supported between instances of 'MagicMock' and 'float'

worker_heavy/simulation/metrics.py:19: TypeError
________________________________________________ test_full_workflow_end_to_end _________________________________________________

self = <Coroutine test_full_workflow_end_to_end>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a52958f860>
coro = <coroutine object test_full_workflow_end_to_end at 0x71a50828f2e0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
_____________________________________ test_genesis_builder_generates_msh_when_fem_enabled ______________________________________

mock_export_stl = <MagicMock name='export_stl' id='124954178210368'>
mock_repair = <MagicMock name='repair_mesh_file' id='124954177240528'>
mock_tetra = <MagicMock name='tetrahedralize' id='124954177039024'>
tmp_path = PosixPath('/tmp/pytest-of-maksym/pytest-2/test_genesis_builder_generates0')

    @patch("worker_heavy.utils.mesh_utils.tetrahedralize")
    @patch("worker_heavy.utils.mesh_utils.repair_mesh_file")
    @patch("worker_heavy.simulation.builder.export_stl", side_effect=create_dummy_stl)
    def test_genesis_builder_generates_msh_when_fem_enabled(
        mock_export_stl, mock_repair, mock_tetra, tmp_path
    ):
        # Setup
        output_dir = tmp_path / "output"
        builder = GenesisSimulationBuilder(output_dir)
    
        # Create a simple assembly
        from shared.models.schemas import PartMetadata
    
        box = Box(10, 10, 10)
        box.label = "test_part"
        box.metadata = PartMetadata(material_id="aluminum-6061")
        assembly = Compound(children=[box])
    
        # Define objectives with FEM enabled
        objectives = ObjectivesYaml(
            objectives=ObjectivesSection(
                goal_zone=BoundingBox(min=(0, 0, 0), max=(1, 1, 1)),
                build_zone=BoundingBox(min=(-10, -10, -10), max=(10, 10, 10)),
            ),
            physics=PhysicsConfig(backend="genesis", fem_enabled=True),
            simulation_bounds=BoundingBox(min=(-10, -10, -10), max=(10, 10, 10)),
            moved_object=MovedObject(
                label="test_part",
                start_position=(0, 0, 0),
                runtime_jitter=(0, 0, 0),
                shape="box",
            ),
            constraints=Constraints(max_unit_cost=100, max_weight_g=10),
        )
    
        # Execute
>       scene_path = builder.build_from_assembly(assembly, objectives=objectives)
                     ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

tests/integration/test_genesis_builder.py:58: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
worker_heavy/simulation/builder.py:995: in build_from_assembly
    mesh = trimesh.load(str(repaired_stl_path))
           ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.venv/lib/python3.12/site-packages/trimesh/exchange/load.py:111: in load
    loaded = load_scene(
.venv/lib/python3.12/site-packages/trimesh/exchange/load.py:193: in load_scene
    arg = _parse_file_args(
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

file_obj = '/tmp/pytest-of-maksym/pytest-2/test_genesis_builder_generates0/output/assets/test_part.repaired.stl'
file_type = None, resolver = None, allow_remote = False, kwargs = {}
file_path = '/tmp/pytest-of-maksym/pytest-2/test_genesis_builder_generates0/output/assets/test_part.repaired.stl'
was_opened = False, exists = False

    def _parse_file_args(
        file_obj,
        file_type: Optional[str],
        resolver: Optional[resolvers.ResolverLike] = None,
        allow_remote: bool = False,
        **kwargs,
    ) -> LoadSource:
        """
        Given a file_obj and a file_type try to magically convert
        arguments to a file-like object and a lowercase string of
        file type.
    
        Parameters
        -----------
        file_obj : str
          if string represents a file path, returns:
            file_obj:   an 'rb' opened file object of the path
            file_type:  the extension from the file path
    
         if string is NOT a path, but has JSON-like special characters:
            file_obj:   the same string passed as file_obj
            file_type:  set to 'json'
    
         if string is a valid-looking URL
            file_obj: an open 'rb' file object with retrieved data
            file_type: from the extension
    
         if string is none of those:
            raise ValueError as we can't do anything with input
    
         if file like object:
            ValueError will be raised if file_type is None
            file_obj:  same as input
            file_type: same as input
    
         if other object: like a shapely.geometry.Polygon, etc:
            file_obj:  same as input
            file_type: if None initially, set to the class name
                        (in lower case), otherwise passed through
    
        file_type : str
             type of file and handled according to above
    
        Returns
        -----------
        args
          Populated `_FileArg` message
        """
        # try to save a file path from various inputs
        file_path = None
    
        # keep track if we opened a file ourselves and thus are
        # responsible for closing it at the end of loading
        was_opened = False
    
        if util.is_pathlib(file_obj):
            # convert pathlib objects to string
            file_obj = str(file_obj.absolute())
    
        if util.is_file(file_obj) and file_type is None:
            raise ValueError("`file_type` must be set for file objects!")
    
        if isinstance(file_obj, str):
            try:
                # clean up file path to an absolute location
                file_path = os.path.abspath(os.path.expanduser(file_obj))
                # check to see if this path exists
                exists = os.path.isfile(file_path)
            except BaseException:
                exists = False
                file_path = None
    
            # file obj is a string which exists on filesystm
            if exists:
                # if not passed create a resolver to find other files
                if resolver is None:
                    resolver = resolvers.FilePathResolver(file_path)
                # save the file name and path to metadata
                # if file_obj is a path that exists use extension as file_type
                if file_type is None:
                    file_type = util.split_extension(file_path, special=["tar.gz", "tar.bz2"])
                # actually open the file
                file_obj = open(file_path, "rb")
                # save that we opened it so we can cleanup later
                was_opened = True
            else:
                if "{" in file_obj:
                    # if a bracket is in the string it's probably straight JSON
                    file_type = "json"
                    file_obj = util.wrap_as_stream(file_obj)
                elif "https://" in file_obj or "http://" in file_obj:
                    if not allow_remote:
                        raise ValueError("unable to load URL with `allow_remote=False`")
    
                    import urllib
    
                    # remove the url-safe encoding and query params
                    file_type = util.split_extension(
                        urllib.parse.unquote(file_obj).split("?", 1)[0].split("/")[-1].strip()
                    )
                    # create a web resolver to do the fetching and whatnot
                    resolver = resolvers.WebResolver(url=file_obj)
                    # fetch the base file
                    file_obj = util.wrap_as_stream(resolver.get_base())
    
                elif file_type is None:
>                   raise ValueError(f"string is not a file: `{file_obj}`")
E                   ValueError: string is not a file: `/tmp/pytest-of-maksym/pytest-2/test_genesis_builder_generates0/output/assets/test_part.repaired.stl`

.venv/lib/python3.12/site-packages/trimesh/exchange/load.py:624: ValueError
------------------------------------------------------ Captured log call -------------------------------------------------------
INFO     build123d:build_common.py:345 None context requested by Box
___________________________________________________ test_gpu_oom_retry_logic ___________________________________________________

tmp_path = PosixPath('/tmp/pytest-of-maksym/pytest-2/test_gpu_oom_retry_logic0')

    @pytest.mark.integration
    def test_gpu_oom_retry_logic(tmp_path):
        """
        Verify that a GPU OOM during Genesis simulation triggers a retry with lower fidelity.
        """
        component = Box(1, 1, 1)
    
>       with (
            patch("worker_heavy.utils.validation.get_simulation_builder") as mock_builder,
            patch("worker_heavy.utils.validation.SimulationLoop") as mock_loop_cls,
            patch("worker_heavy.utils.validation.prerender_24_views") as mock_render,
            patch("worker_heavy.utils.validation.calculate_assembly_totals") as mock_totals,
        ):

tests/integration/test_gpu_oom_retry.py:17: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
/usr/lib/python3.12/unittest/mock.py:1458: in __enter__
    original, local = self.get_original()
                      ^^^^^^^^^^^^^^^^^^^
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <unittest.mock._patch object at 0x71a5295c0050>

    def get_original(self):
        target = self.getter()
        name = self.attribute
    
        original = DEFAULT
        local = False
    
        try:
            original = target.__dict__[name]
        except (AttributeError, KeyError):
            original = getattr(target, name, DEFAULT)
        else:
            local = True
    
        if name in _builtins and isinstance(target, ModuleType):
            self.create = True
    
        if not self.create and original is DEFAULT:
>           raise AttributeError(
                "%s does not have the attribute %r" % (target, name)
            )
E           AttributeError: <module 'worker_heavy.utils.validation' from '/home/maksym/Work/proj/Problemologist/Problemologist-AI/worker_heavy/utils/validation.py'> does not have the attribute 'SimulationLoop'

/usr/lib/python3.12/unittest/mock.py:1431: AttributeError
------------------------------------------------------ Captured log call -------------------------------------------------------
INFO     build123d:build_common.py:345 None context requested by Box
___________________________________________________ test_multitenancy_repro ____________________________________________________

self = <Coroutine test_multitenancy_repro>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5297e22a0>, coro = <coroutine object test_multitenancy_repro at 0x71a591ffbde0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
__________________________________________________ test_flow_rate_integration __________________________________________________

genesis_backend = <worker_heavy.simulation.genesis_backend.GenesisBackend object at 0x71a52973f380>
tmp_path = PosixPath('/tmp/pytest-of-maksym/pytest-2/test_flow_rate_integration0')

    def test_flow_rate_integration(genesis_backend, tmp_path):
        # Mock scene build status
        type(genesis_backend.scene).is_built = PropertyMock(return_value=True)
    
        # 1. Define objectives with flow rate
        # Gate plane: center (0, 0, 0), normal (0, 0, 1) - horizontal plane
        flow_obj = FlowRateObjective(
            fluid_id="water",
            gate_plane_point=(0, 0, 0),
            gate_plane_normal=(0, 0, 1),
            target_rate_l_per_s=0.01,  # 10ml/s
            tolerance=0.2,
        )
    
        objectives = ObjectivesYaml(
            objectives=ObjectivesSection(
                goal_zone=BoundingBox(min=(10, 10, 10), max=(11, 11, 11)),
                fluid_objectives=[flow_obj],
                build_zone=BoundingBox(min=(-10, -10, -10), max=(10, 10, 10)),
            ),
            physics=PhysicsConfig(backend="genesis"),
            simulation_bounds=BoundingBox(min=(-10, -10, -10), max=(10, 10, 10)),
            moved_object=MovedObject(
                label="target",
                start_position=(0, 0, 0),
                runtime_jitter=(0, 0, 0),
                shape="box",
            ),
            constraints=Constraints(max_unit_cost=100, max_weight_g=10),
        )
    
        # 2. Setup mock particles: moving across Z=0 plane
        # 10 particles crossing per step.
    
        def mock_get_particles():
            # particles start below plane
            if mock_step.time == 0:
                return np.array([[0, 0, -0.1]] * 100)
            # move above plane
            return np.array([[0, 0, 0.1]] * 100)
    
        # 3. Setup Loop with mocked backend
        from shared.models.simulation import SimulationMetrics as SharedSimulationMetrics
    
        with (
            patch("worker_heavy.simulation.loop.get_physics_backend") as mock_get,
            patch(
                "worker_heavy.simulation.loop.SimulationMetrics",
                new=SharedSimulationMetrics,
            ),
        ):
            mock_get.return_value = genesis_backend
    
            # Ensure methods are mocks
            genesis_backend.step = MagicMock()
            genesis_backend.get_particle_positions = MagicMock()
            genesis_backend.get_particle_positions.side_effect = mock_get_particles
            genesis_backend.get_all_body_names = MagicMock(return_value=["world"])
            genesis_backend.get_stress_summaries = MagicMock(return_value=[])
            genesis_backend.get_stress_field = MagicMock(return_value=None)
    
            xml_path = tmp_path / "scene.json"
            xml_path.write_text("{}")  # Dummy JSON
    
            def mock_step(dt):
                mock_step.time += dt
                return StepResult(time=mock_step.time, success=True)
    
            mock_step.time = 0.0
            genesis_backend.step.side_effect = mock_step
    
            loop = SimulationLoop(
                str(xml_path),
                backend_type=SimulatorBackendType.GENESIS,
                objectives=objectives,
            )
    
            # Run for 0.1 seconds (50 steps)
            metrics = loop.step({}, duration=0.1)
    
            # 4. Verify results
            # NOTE: Due to a bug in SimulationLoop.step, it currently returns success=True even if objectives fail.
            # NOTE: Due to a bug in SimulationLoop.step, it currently returns success=True even if objectives fail.
            assert metrics.success is True
            assert len(metrics.fluid_metrics) == 1
            assert metrics.fluid_metrics[0].fluid_id == "water"
            assert metrics.fluid_metrics[0].metric_type == "flow_rate"
>           assert metrics.fluid_metrics[0].passed is True
E           AssertionError: assert False is True
E            +  where False = FluidMetricResult(metric_type='flow_rate', fluid_id='water', measured_value=0.0, target_value=0.01, passed=False).passed

tests/integration/test_physics_fluids_wp04.py:133: AssertionError
________________________________________________ test_fluid_containment_failure ________________________________________________

mock_genesis_backend = <MagicMock name='get_physics_backend()' id='124956535566224'>
tmp_path = PosixPath('/tmp/pytest-of-maksym/pytest-2/test_fluid_containment_failure0')

    def test_fluid_containment_failure(mock_genesis_backend, tmp_path):
        # 1. Define objectives
        containment_obj = FluidContainmentObjective(
            fluid_id="water",
            containment_zone=BoundingBox(min=(0, 0, 0), max=(1, 1, 1)),
            threshold=0.9,
        )
    
        objectives = ObjectivesYaml(
            objectives=ObjectivesSection(
                goal_zone=BoundingBox(min=(10, 10, 10), max=(11, 11, 11)),
                fluid_objectives=[containment_obj],
                build_zone=BoundingBox(min=(-10, -10, -10), max=(10, 10, 10)),
            ),
            physics=PhysicsConfig(backend="genesis"),
            simulation_bounds=BoundingBox(min=(-10, -10, -10), max=(10, 10, 10)),
            moved_object=MovedObject(
                label="target",
                start_position=(0, 0, 0),
                runtime_jitter=(0, 0, 0),
                shape="box",
            ),
            constraints=Constraints(max_unit_cost=100, max_weight_g=10),
        )
    
        # 2. Setup mock particles: 50 inside, 50 outside (ratio 0.5 < 0.9)
        inside = np.random.uniform(0.1, 0.9, (50, 3))
        outside = np.random.uniform(2.0, 3.0, (50, 3))
        mock_genesis_backend.get_particle_positions.return_value = np.vstack(
            [inside, outside]
        )
        mock_genesis_backend.get_stress_field.return_value = StressField(
            nodes=np.zeros((1, 3)), stress=np.array([100e6])
        )
    
        xml_path = tmp_path / "scene.xml"
        xml_path.write_text("<mujoco/>")
    
        loop = SimulationLoop(
            str(xml_path),
            backend_type=SimulatorBackendType.GENESIS,
            objectives=objectives,
            smoke_test_mode=True,
        )
    
        def mock_step(dt):
            mock_step.time += dt
            return StepResult(time=mock_step.time, success=True)
    
        mock_step.time = 0.0
        mock_genesis_backend.step.side_effect = mock_step
    
        # Run simulation
        metrics = loop.step({}, duration=0.1)
    
        # 3. Verify failure
>       assert metrics.success is False
E       AssertionError: assert True is False
E        +  where True = SimulationMetrics(total_time=1.0, total_energy=0.0, max_velocity=0.0, max_stress=0.0, success=True, fail_reason=None, fail_mode=None, failure=None, stress_summaries=[], stress_fields={}, fluid_metrics=[FluidMetricResult(metric_type='fluid_containment', fluid_id='water', measured_value=0.5, target_value=0.9, passed=False)], events=[], confidence='approximate').success

tests/integration/test_physics_genesis.py:213: AssertionError
__________________________________________ test_simulation_concurrency_serialization ___________________________________________

self = <Coroutine test_simulation_concurrency_serialization>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5706b2c90>
coro = <coroutine object test_simulation_concurrency_serialization at 0x71a590add6c0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
___________________________________________________ test_worker_concurrency ____________________________________________________

self = <Coroutine test_worker_concurrency>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a529370080>, coro = <coroutine object test_worker_concurrency at 0x71a5b196c190>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
______________________________________________ test_report_trace_feedback_success ______________________________________________

self = <Coroutine test_report_trace_feedback_success>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a529371a60>
coro = <coroutine object test_report_trace_feedback_success at 0x71a5b0a6bcc0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
__________________________________________ test_report_trace_feedback_no_langfuse_id ___________________________________________

self = <Coroutine test_report_trace_feedback_no_langfuse_id>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5705a4200>
coro = <coroutine object test_report_trace_feedback_no_langfuse_id at 0x71a5908ae400>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
__________________________________________ test_calculate_and_report_automated_score ___________________________________________

self = <Coroutine test_calculate_and_report_automated_score>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5706b3cb0>
coro = <coroutine object test_calculate_and_report_automated_score at 0x71a5b43f9170>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
____________________________________________________ test_setup_persistence ____________________________________________________

self = <Coroutine test_setup_persistence>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a741149520>, coro = <coroutine object test_setup_persistence at 0x71a5905c9e40>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
__________________________________________________ test_async_upload_download __________________________________________________

self = <Coroutine test_async_upload_download>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a4f02dcfb0>, coro = <coroutine object test_async_upload_download at 0x71a57098fdf0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
------------------------------------------------------ Captured log setup ------------------------------------------------------
INFO     botocore.credentials:credentials.py:1252 Found credentials in environment variables.
____________________________________________ test_nodes_call_get_langfuse_callback _____________________________________________

self = <Coroutine test_nodes_call_get_langfuse_callback>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a4fea577d0>
coro = <coroutine object test_nodes_call_get_langfuse_callback at 0x71a4fffaf0f0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
___________________________________________ test_graph_initializes_langfuse_callback ___________________________________________

self = <Coroutine test_graph_initializes_langfuse_callback>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a4e4438080>
coro = <coroutine object test_graph_initializes_langfuse_callback at 0x71a4ff1d7880>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
____________________________________________________ test_websocket_manager ____________________________________________________

    def test_websocket_manager():
        from controller.api.manager import ConnectionManager
    
        manager = ConnectionManager()
    
        mock_ws = AsyncMock()
        episode_id = uuid.uuid4()
    
        # Test connect
        # manager.connect(episode_id, mock_ws) is async
        import asyncio
    
>       asyncio.run(manager.connect(episode_id, mock_ws))

tests/test_controller_api_extended.py:128: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

main = <coroutine object ConnectionManager.connect at 0x71a4ff3f2c50>

    def run(main, *, debug=None, loop_factory=None):
        """Execute the coroutine and return the result.
    
        This function runs the passed coroutine, taking care of
        managing the asyncio event loop, finalizing asynchronous
        generators and closing the default executor.
    
        This function cannot be called when another asyncio event loop is
        running in the same thread.
    
        If debug is True, the event loop will be run in debug mode.
    
        This function always creates a new event loop and closes it at the end.
        It should be used as a main entry point for asyncio programs, and should
        ideally only be called once.
    
        The executor is given a timeout duration of 5 minutes to shutdown.
        If the executor hasn't finished within that duration, a warning is
        emitted and the executor is closed.
    
        Example:
    
            async def main():
                await asyncio.sleep(1)
                print('hello')
    
            asyncio.run(main())
        """
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "asyncio.run() cannot be called from a running event loop")
E           RuntimeError: asyncio.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:190: RuntimeError
_______________________________________________ test_websocket_broadcast_failure _______________________________________________

    def test_websocket_broadcast_failure():
        from controller.api.manager import ConnectionManager
    
        manager = ConnectionManager()
    
        mock_ws = AsyncMock()
        mock_ws.send_json.side_effect = Exception("Send failed")
        episode_id = uuid.uuid4()
    
        import asyncio
    
>       asyncio.run(manager.connect(episode_id, mock_ws))

tests/test_controller_api_extended.py:152: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

main = <coroutine object ConnectionManager.connect at 0x71a52992bd30>

    def run(main, *, debug=None, loop_factory=None):
        """Execute the coroutine and return the result.
    
        This function runs the passed coroutine, taking care of
        managing the asyncio event loop, finalizing asynchronous
        generators and closing the default executor.
    
        This function cannot be called when another asyncio event loop is
        running in the same thread.
    
        If debug is True, the event loop will be run in debug mode.
    
        This function always creates a new event loop and closes it at the end.
        It should be used as a main entry point for asyncio programs, and should
        ideally only be called once.
    
        The executor is given a timeout duration of 5 minutes to shutdown.
        If the executor hasn't finished within that duration, a warning is
        emitted and the executor is closed.
    
        Example:
    
            async def main():
                await asyncio.sleep(1)
                print('hello')
    
            asyncio.run(main())
        """
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "asyncio.run() cannot be called from a running event loop")
E           RuntimeError: asyncio.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:190: RuntimeError
_______________________________________________ test_execute_agent_task_success ________________________________________________

self = <Coroutine test_execute_agent_task_success>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a4fe7f39e0>
coro = <coroutine object test_execute_agent_task_success at 0x71a570109ad0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
______________________________________ test_execute_agent_task_without_langfuse_callback _______________________________________

self = <Coroutine test_execute_agent_task_without_langfuse_callback>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a4fe655c70>
coro = <coroutine object test_execute_agent_task_without_langfuse_callback at 0x71a5701086a0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
_______________________________________________ test_search_cots_catalog_success _______________________________________________

mock_search_parts = <MagicMock name='search_parts' id='124955367212144'>

    def test_search_cots_catalog_success(mock_search_parts):
        # Setup mock return data
        mock_search_parts.return_value = [
            COTSItem(
                part_id="M3-10",
                name="M3 Bolt 10mm",
                category="fastener",
                unit_cost=0.10,
                weight_g=1.5,
                import_recipe="Box(3)",
                metadata={"material": "steel"},
            ),
            COTSItem(
                part_id="M3-12",
                name="M3 Bolt 12mm",
                category="fastener",
                unit_cost=0.12,
                weight_g=1.7,
                import_recipe="Box(3)",
                metadata={"material": "steel"},
            ),
        ]
    
        # Invoke tool with correct arguments dict
>       result = search_cots_catalog.invoke({"query": "M3 bolts", "max_cost": 0.20})
                 ^^^^^^^^^^^^^^^^^^^^^^^^^^
E       AttributeError: 'function' object has no attribute 'invoke'

tests/test_cots_search.py:39: AttributeError
_____________________________________________ test_search_cots_catalog_no_results ______________________________________________

mock_search_parts = <MagicMock name='search_parts' id='124953456625184'>

    def test_search_cots_catalog_no_results(mock_search_parts):
        mock_search_parts.return_value = []
    
>       result = search_cots_catalog.invoke({"query": "Unobtainium"})
                 ^^^^^^^^^^^^^^^^^^^^^^^^^^
E       AttributeError: 'function' object has no attribute 'invoke'

tests/test_cots_search.py:57: AttributeError
______________________________________________ test_search_cots_catalog_all_args _______________________________________________

mock_search_parts = <MagicMock name='search_parts' id='124953458313984'>

    def test_search_cots_catalog_all_args(mock_search_parts):
        mock_search_parts.return_value = []
    
>       search_cots_catalog.invoke(
        ^^^^^^^^^^^^^^^^^^^^^^^^^^
            {
                "query": "Motor",
                "max_weight_g": 100.0,
                "max_cost": 50.0,
                "category": "motor",
                "limit": 10,
            }
        )
E       AttributeError: 'function' object has no attribute 'invoke'

tests/test_cots_search.py:65: AttributeError
_____________________________________________________ test_services_health _____________________________________________________

self = <Coroutine test_services_health>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a4feaf7b60>, coro = <coroutine object test_services_health at 0x71a59097dbd0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
_____________________________________________ test_controller_to_worker_agent_run ______________________________________________

self = <Coroutine test_controller_to_worker_agent_run>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a5b12acb90>
coro = <coroutine object test_controller_to_worker_agent_run at 0x71a529f61b40>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
______________________________________________ test_execute_agent_task_cancelled _______________________________________________

self = <Coroutine test_execute_agent_task_cancelled>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a4fe9c2300>
coro = <coroutine object test_execute_agent_task_cancelled at 0x71a59087d250>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
________________________________________________ test_interrupt_episode_success ________________________________________________

self = <Coroutine test_interrupt_episode_success>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a4fe67efc0>
coro = <coroutine object test_interrupt_episode_success at 0x71a50831c8e0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
________________________________________________________ test_db_setup _________________________________________________________

self = <Coroutine test_db_setup>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a4fe8f0c20>, coro = <coroutine object test_db_setup at 0x71a590671cc0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
___________________________________________________ test_episode_broadcaster ___________________________________________________

self = <Coroutine test_episode_broadcaster>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a4fe8f08c0>, coro = <coroutine object test_episode_broadcaster at 0x71a509bdb380>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
______________________________________________________ test_get_asset_glb ______________________________________________________

self = <Coroutine test_get_asset_glb>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a4fe8f0230>, coro = <coroutine object test_get_asset_glb at 0x71a5096f2d40>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
______________________________________________________ test_get_asset_py _______________________________________________________

self = <Coroutine test_get_asset_py>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a4fe67e4e0>, coro = <coroutine object test_get_asset_py at 0x71a5708d4bf0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
_________________________________________________ test_get_asset_syntax_error __________________________________________________

self = <Coroutine test_get_asset_syntax_error>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a4fe67dd60>
coro = <coroutine object test_get_asset_syntax_error at 0x71a590671cc0>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
______________________________________________ test_remote_fs_middleware_temporal ______________________________________________

self = <Coroutine test_remote_fs_middleware_temporal>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a4fe8f02f0>
coro = <coroutine object test_remote_fs_middleware_temporal at 0x71a5096f1120>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
_______________________________________________________ test_load_config _______________________________________________________

    def test_load_config():
        with capture_logs() as captured:
            config = load_config()
>           assert any(log["event"] == "loading_manufacturing_config" for log in captured)
E           assert False
E            +  where False = any(<generator object test_load_config.<locals>.<genexpr> at 0x71a518570930>)

tests/workbenches/test_config.py:11: AssertionError
_____________________________________________________ test_3dp_valid_part ______________________________________________________

config = ManufacturingConfig(defaults={'currency': 'USD'}, materials={'aluminum_6061': MaterialDefinition(name='Aluminum 6061',...pa=None, elongation_at_break=None, material_class='rigid')}, constraints={}, parameters={}, costs={'setup_fee': 10.0}))

    def test_3dp_valid_part(config):
        part = Box(10, 10, 10)
        result = analyze_3dp(part, config)
        assert result.is_manufacturable is True
        assert len(result.violations) == 0
        assert result.unit_cost > 0
>       assert "cost_breakdown" in result.metadata
E       AssertionError: assert 'cost_breakdown' in WorkbenchMetadata(cost_breakdown=CostBreakdown(process='print_3d', total_cost=11.354133333333333, unit_cost=11.354133333333333, material_cost_per_unit=0.0208, setup_cost=10.0, is_reused=False, details={'part_volume_cm3': 1.0, 'printing_time_hr': 0.07, 'run_cost_per_unit': 1.3333}, pricing_explanation='3DP cost ($11.35) for 1 units. Material: abs ($0.0208/unit). Print time: 0.07 hr. Setup fee: $10.00.'), dof_count=None, dof_warning=None, undercut_count=None, gate_count=None, additional_info={})
E        +  where WorkbenchMetadata(cost_breakdown=CostBreakdown(process='print_3d', total_cost=11.354133333333333, unit_cost=11.354133333333333, material_cost_per_unit=0.0208, setup_cost=10.0, is_reused=False, details={'part_volume_cm3': 1.0, 'printing_time_hr': 0.07, 'run_cost_per_unit': 1.3333}, pricing_explanation='3DP cost ($11.35) for 1 units. Material: abs ($0.0208/unit). Print time: 0.07 hr. Setup fee: $10.00.'), dof_count=None, dof_warning=None, undercut_count=None, gate_count=None, additional_info={}) = WorkbenchResult(is_manufacturable=True, unit_cost=11.354133333333333, weight_g=1.0399999999999998, violations=[], metadata=WorkbenchMetadata(cost_breakdown=CostBreakdown(process='print_3d', total_cost=11.354133333333333, unit_cost=11.354133333333333, material_cost_per_unit=0.0208, setup_cost=10.0, is_reused=False, details={'part_volume_cm3': 1.0, 'printing_time_hr': 0.07, 'run_cost_per_unit': 1.3333}, pricing_explanation='3DP cost ($11.35) for 1 units. Material: abs ($0.0208/unit). Print time: 0.07 hr. Setup fee: $10.00.'), dof_count=None, dof_warning=None, undercut_count=None, gate_count=None, additional_info={})).metadata

tests/workbenches/test_print_3d.py:19: AssertionError
------------------------------------------------------ Captured log call -------------------------------------------------------
INFO     build123d:build_common.py:345 None context requested by Box
___________________________________________________ test_dynamic_controllers ___________________________________________________

sim_loop = <worker_heavy.simulation.loop.SimulationLoop object at 0x71a529e5ff80>

    def test_dynamic_controllers(sim_loop):
        # Constant 10.0 controller
        controllers = {"test_actuator": constant(10.0)}
    
        # Run for a very short time
        sim_loop.step({}, duration=0.01, dynamic_controllers=controllers)
    
        # Check that ctrl was set to 10.0
        # Check that ctrl was set to 10.0
        state = sim_loop.backend.get_actuator_state("test_actuator")
>       assert state.ctrl == 10.0
E       assert 0.0 == 10.0
E        +  where 0.0 = ActuatorState(force=0.0, velocity=0.0, ctrl=0.0, forcerange=(0.0, 0.0)).ctrl

tests/worker_heavy/simulation/test_dynamic_control.py:39: AssertionError
___________________________________________________ test_metrics_collection ____________________________________________________

sim_loop = <worker_heavy.simulation.loop.SimulationLoop object at 0x71a570305670>

    def test_metrics_collection(sim_loop):
        # Apply control to actuator
        # Gear is 1, so control 1.0 should apply 1.0 torque
        metrics = sim_loop.step({"test_actuator": 1.0}, duration=0.5)
    
        assert metrics.total_energy > 0
        assert metrics.max_velocity >= 0  # Target box might not move much but hinges will
    
        # Check velocity of target box specifically
        sim_loop.backend.data.qvel[0] = 5.0  # Set x-velocity of target box
        metrics = sim_loop.step({}, duration=0.01)
>       assert metrics.max_velocity >= 5.0
E       AssertionError: assert 0.0 >= 5.0
E        +  where 0.0 = SimulationMetrics(total_time=0.0, total_energy=0.0, max_velocity=0.0, max_stress=0.0, success=False, fail_reason=None, fail_mode=None, failure=None, stress_summaries=[], stress_fields={}, fluid_metrics=[], events=[], confidence='approximate').max_velocity

tests/worker_heavy/simulation/test_loop.py:74: AssertionError
____________________________________________________ test_goal_zone_trigger ____________________________________________________

sim_loop = <worker_heavy.simulation.loop.SimulationLoop object at 0x71a5299b0320>

    def test_goal_zone_trigger(sim_loop):
        # move target to goal
        # Goal is at 0.5 0 0
        # Target is free joint. qpos[0:3] is pos.
        sim_loop.backend.data.qpos[0] = 0.5
        sim_loop.backend.data.qpos[1] = 0.0
        sim_loop.backend.data.qpos[2] = 0.0
    
        # Run step
        metrics = sim_loop.step({}, duration=0.01)
>       assert metrics.success is True
E       AssertionError: assert False is True
E        +  where False = SimulationMetrics(total_time=0.0, total_energy=0.0, max_velocity=0.0, max_stress=0.0, success=False, fail_reason=None, fail_mode=None, failure=None, stress_summaries=[], stress_fields={}, fluid_metrics=[], events=[], confidence='approximate').success

tests/worker_heavy/simulation/test_loop.py:87: AssertionError
_________________________________________________ test_forbidden_zone_trigger __________________________________________________

sim_loop = <worker_heavy.simulation.loop.SimulationLoop object at 0x71a5299b0380>

    def test_forbidden_zone_trigger(sim_loop):
        # Move target to forbidden zone (-0.5 0 0)
        sim_loop.backend.data.qpos[0] = -0.5
        sim_loop.backend.data.qpos[1] = 0.0
        sim_loop.backend.data.qpos[2] = 0.0
    
        # Run step
        metrics = sim_loop.step({}, duration=0.01)
    
        # Collision detection relies on contacts.
        # Just setting position might not generate contacts immediately if no step is running?
        # instability_detection usually returns physics_instability
>       assert metrics.fail_mode == FailureReason.FORBID_ZONE_HIT
E       AssertionError: assert None == <FailureReason.FORBID_ZONE_HIT: 'FORBID_ZONE_HIT'>
E        +  where None = SimulationMetrics(total_time=0.0, total_energy=0.0, max_velocity=0.0, max_stress=0.0, success=False, fail_reason=None, fail_mode=None, failure=None, stress_summaries=[], stress_fields={}, fluid_metrics=[], events=[], confidence='approximate').fail_mode
E        +  and   <FailureReason.FORBID_ZONE_HIT: 'FORBID_ZONE_HIT'> = FailureReason.FORBID_ZONE_HIT

tests/worker_heavy/simulation/test_loop.py:102: AssertionError
__________________________________________________ test_target_fell_off_world __________________________________________________

sim_loop = <worker_heavy.simulation.loop.SimulationLoop object at 0x71a5705a5130>

    def test_target_fell_off_world(sim_loop):
        """Test failure detection when target falls below Z threshold."""
        # Set target Z to -6.0 (below default -5.0 threshold)
        sim_loop.backend.data.qpos[2] = -6.0
    
        # Run step
        metrics = sim_loop.step({}, duration=0.01)
    
        assert metrics.success is False
>       assert metrics.fail_mode == FailureReason.OUT_OF_BOUNDS
E       AssertionError: assert None == <FailureReason.OUT_OF_BOUNDS: 'OUT_OF_BOUNDS'>
E        +  where None = SimulationMetrics(total_time=0.0, total_energy=0.0, max_velocity=0.0, max_stress=0.0, success=False, fail_reason=None, fail_mode=None, failure=None, stress_summaries=[], stress_fields={}, fluid_metrics=[], events=[], confidence='approximate').fail_mode
E        +  and   <FailureReason.OUT_OF_BOUNDS: 'OUT_OF_BOUNDS'> = FailureReason.OUT_OF_BOUNDS

tests/worker_heavy/simulation/test_loop.py:173: AssertionError
__________________________________________________ test_instability_detection __________________________________________________

sim_loop = <worker_heavy.simulation.loop.SimulationLoop object at 0x71a5706b05f0>

    def test_instability_detection(sim_loop):
        """Test failure detection when NaNs appear in simulation state."""
        import numpy as np
    
        # Inject NaN into qpos
        sim_loop.backend.data.qpos[0] = np.nan
    
        # Run step
        metrics = sim_loop.step({}, duration=0.01)
    
        assert metrics.success is False
>       assert metrics.fail_mode == FailureReason.PHYSICS_INSTABILITY
E       AssertionError: assert None == <FailureReason.PHYSICS_INSTABILITY: 'PHYSICS_INSTABILITY'>
E        +  where None = SimulationMetrics(total_time=0.0, total_energy=0.0, max_velocity=0.0, max_stress=0.0, success=False, fail_reason=None, fail_mode=None, failure=None, stress_summaries=[], stress_fields={}, fluid_metrics=[], events=[], confidence='approximate').fail_mode
E        +  and   <FailureReason.PHYSICS_INSTABILITY: 'PHYSICS_INSTABILITY'> = FailureReason.PHYSICS_INSTABILITY

tests/worker_heavy/simulation/test_loop.py:187: AssertionError
_______________________________________ TestSimulationLoopStress.test_stress_collection ________________________________________

self = <tests.worker_heavy.simulation.test_loop_stress.TestSimulationLoopStress object at 0x71a5b1c389b0>
mock_get_backend = <MagicMock name='get_physics_backend' id='124955368509744'>

    @patch("worker_heavy.simulation.loop.get_physics_backend")
    def test_stress_collection(self, mock_get_backend):
        # Setup mock backend
        mock_backend = MagicMock()
        # Delete attributes that SimulationLoop checks via hasattr() to avoid
        # MagicMock returning truthy values for them by default.
        del mock_backend.timestep
        del mock_backend.model
        mock_get_backend.return_value = mock_backend
    
        mock_backend.get_all_site_names.return_value = []
        mock_backend.get_all_actuator_names.return_value = []
        mock_backend.get_all_body_names.return_value = ["target_box"]
        mock_backend.get_body_state.return_value = MagicMock(
            pos=(0, 0, 0), vel=(0, 0, 0)
        )
        mock_backend.step.return_value = MagicMock(time=0.1, success=True)
    
        # Define sequential stress values to return
        mock_backend.get_max_stress.side_effect = [100.0, 500.0, 300.0]
    
        loop = SimulationLoop(
            xml_path="dummy.xml", backend_type=SimulatorBackendType.GENESIS
        )
    
        # Run 3 steps
        # SimulationLoop uses dt=0.002
        # duration=0.006 should result in 3 steps
        metrics = loop.step(control_inputs={}, duration=0.006)
    
        # Verify get_max_stress was called
>       assert mock_backend.get_max_stress.call_count >= 3
E       AssertionError: assert 0 >= 3
E        +  where 0 = <MagicMock name='get_physics_backend().get_max_stress' id='124953455382112'>.call_count
E        +    where <MagicMock name='get_physics_backend().get_max_stress' id='124953455382112'> = <MagicMock name='get_physics_backend()' id='124955368509504'>.get_max_stress

tests/worker_heavy/simulation/test_loop_stress.py:39: AssertionError
______________________________________ TestMotorOverload.test_overload_detection_triggers ______________________________________

self = <tests.worker_heavy.simulation.test_motor_overload.TestMotorOverload object at 0x71a5b1c394c0>
overload_loop = <worker_heavy.simulation.loop.SimulationLoop object at 0x71a4fc3f5fd0>

    def test_overload_detection_triggers(self, overload_loop):
        """Test that motor clamped at forcerange for >2s fails simulation."""
        # Demand large position that can't be reached with tiny forcerange
        # This will keep the motor saturated
        from worker_heavy.utils.controllers.position_based import hold_position
    
        controllers = {"servo": hold_position(3.14)}  # Impossible with 0.1 N limit
    
        # Run for 3 seconds (should trigger 2s overload threshold)
        metrics = overload_loop.step({}, duration=3.0, dynamic_controllers=controllers)
    
        assert not metrics.success
        assert metrics.fail_reason is not None
        # Architecture spec uses 'motor_overload'
>       assert "motor_overload" in metrics.fail_reason
E       AssertionError: assert 'motor_overload' in 'MOTOR_OVERLOAD:servo'
E        +  where 'MOTOR_OVERLOAD:servo' = SimulationMetrics(total_time=2.000000000000001, total_energy=12.246000000000008, max_velocity=0.0, max_stress=0.0, success=False, fail_reason='MOTOR_OVERLOAD:servo', fail_mode=<FailureReason.MOTOR_OVERLOAD: 'MOTOR_OVERLOAD'>, failure=SimulationFailure(reason=<FailureReason.MOTOR_OVERLOAD: 'MOTOR_OVERLOAD'>, detail='servo'), stress_summaries=[], stress_fields={}, fluid_metrics=[], events=[], confidence='approximate').fail_reason

tests/worker_heavy/simulation/test_motor_overload.py:75: AssertionError
______________________________________________________ test_apply_control ______________________________________________________

genesis_backend = <worker_heavy.simulation.genesis_backend.GenesisBackend object at 0x71a4fe6f5190>

    def test_apply_control(genesis_backend):
        mock_entity = MagicMock()
        genesis_backend.entities = {"motor1": mock_entity}
        genesis_backend.motors = [{"part_name": "motor1"}]
    
        genesis_backend.apply_control({"motor1": 10.0})
    
        # Check if set_dofs_force was called with a numpy array containing 10.0
>       args, kwargs = mock_entity.set_dofs_force.call_args
        ^^^^^^^^^^^^
E       TypeError: cannot unpack non-iterable NoneType object

tests/worker_heavy/test_genesis_interaction.py:26: TypeError
_______________________________________________________ test_simulation ________________________________________________________

    @pytest.mark.integration
    @pytest.mark.xdist_group(name="physics_sims")
    def test_simulation():
        # Valid stable box
        with BuildPart() as p:
            Box(10, 10, 10)
    
>       res = simulate(p.part)
              ^^^^^^^^^^^^^^^^

tests/worker_heavy/test_validation.py:69: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
worker_heavy/utils/validation.py:533: in simulate
    scene_path = builder.build_from_assembly(
worker_heavy/simulation/builder.py:920: in build_from_assembly
    parts_data = CommonAssemblyTraverser.traverse(assembly, electronics)
                 ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
worker_heavy/simulation/builder.py:67: in traverse
    meta = CommonAssemblyTraverser._resolve_part_metadata(child)
           ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

child = Part at 0x71a4ff199cd0, label(), #children(0)

    @staticmethod
    def _resolve_part_metadata(child: Any) -> dict[str, Any]:
        from shared.models.schemas import CompoundMetadata, PartMetadata
    
        metadata = getattr(child, "metadata", None)
        if metadata is None:
            # Check for legacy zone prefix - zones are handled separately
            label = getattr(child, "label", "")
            if label.startswith("zone_"):
                return {
                    "is_fixed": True,
                    "material_id": None,
                    "cots_id": None,
                    "joint_type": None,
                    "joint_axis": None,
                    "joint_range": None,
                }
    
>           raise ValueError(
                f"Part '{label or 'unknown'}' is missing required metadata. "
                "Every part must have a .metadata attribute "
                "(PartMetadata or CompoundMetadata)."
            )
E           ValueError: Part 'unknown' is missing required metadata. Every part must have a .metadata attribute (PartMetadata or CompoundMetadata).

worker_heavy/simulation/builder.py:132: ValueError
------------------------------------------------------ Captured log call -------------------------------------------------------
INFO     build123d:build_common.py:345 None context requested by None
INFO     build123d:build_common.py:279 Entering BuildPart with mode=Mode.ADD which is in different scope as parent
INFO     build123d:build_common.py:1275 WorkplaneList is pushing 1 workplanes: [Plane(o=(0.00, 0.00, 0.00), x=(1.00, 0.00, 0.00), z=(0.00, 0.00, 1.00))]
INFO     build123d:build_common.py:887 LocationList is pushing 1 points: [(p=(0.00, 0.00, 0.00), o=(-0.00, 0.00, -0.00))]
INFO     build123d:build_common.py:345 BuildPart context requested by Box
INFO     build123d:build_common.py:491 Completed integrating 1 object(s) into part with Mode=Mode.ADD
INFO     build123d:build_common.py:898 LocationList is popping 1 points
INFO     build123d:build_common.py:1288 WorkplaneList is popping 1 workplanes
INFO     build123d:build_common.py:321 Exiting BuildPart
________________________________________________ test_wire_clearance_violation _________________________________________________

mock_validate_circuit = <MagicMock name='validate_circuit' id='124955364026528'>
mock_get_backend = <MagicMock name='get_physics_backend' id='124954169724176'>

    @patch("worker_heavy.simulation.loop.get_physics_backend")
    @patch("shared.pyspice_utils.validate_circuit")
    def test_wire_clearance_violation(mock_validate_circuit, mock_get_backend):
        # Setup mock backend
        mock_backend = MagicMock()
        mock_get_backend.return_value = mock_backend
        mock_backend.get_all_site_names.return_value = []
        mock_backend.get_all_actuator_names.return_value = []
        mock_backend.get_all_body_names.return_value = []
    
        # 1. Create a Box centered at origin (10x10x10)
        # This box occupies -5 to +5 in X, Y, Z
        component = Box(10, 10, 10)
    
        # 2. Define a wire that goes through the box
        # From (-10, 0, 0) to (10, 0, 0). Passes through (-5, 0, 0) to (5, 0, 0).
        wire = WireConfig(
            wire_id="wire_1",
            from_terminal=WireTerminal(component="src", terminal="t1"),
            to_terminal=WireTerminal(component="dst", terminal="t1"),
            gauge_awg=20,
            length_mm=20.0,
            waypoints=[(-10.0, 0.0, 0.0), (10.0, 0.0, 0.0)],
            routed_in_3d=True,
        )
    
        electronics = ElectronicsSection(
            power_supply=PowerSupplyConfig(voltage_dc=12.0, max_current_a=1.0),
            wiring=[wire],
            components=[
                ElectronicComponent(
                    component_id="src", type=ElectronicComponentType.CONNECTOR
                ),
                ElectronicComponent(
                    component_id="dst", type=ElectronicComponentType.CONNECTOR
                ),
            ],
        )
    
        # Mock validate_circuit to avoid libngspice dependency and ensure valid circuit
        mock_validate_circuit.return_value = MagicMock(
            valid=True, errors=[], node_voltages={}
        )
    
        # 3. Initialize SimulationLoop
        # Note: We need to mock validate_and_price because it requires config loading
        with patch("worker_heavy.simulation.loop.validate_and_price") as mock_validate:
            mock_validate.return_value = MagicMock(is_manufacturable=True)
            # Mock load_config as well since it's called if custom config exists or fallback
            with patch("worker_heavy.simulation.loop.load_config"):
                loop = SimulationLoop(
                    xml_path="dummy.xml",
                    component=component,
                    electronics=electronics,
                    smoke_test_mode=True,
                )
    
                # 4. Check if initialization detected the error
                assert loop.wire_clearance_error is not None
                assert "Wire clearance violation detected" in loop.wire_clearance_error
    
                # 5. Run step and verify fail_reason
                metrics = loop.step(control_inputs={})
                assert metrics.success is False
>               assert metrics.fail_reason == loop.wire_clearance_error
E               AssertionError: assert 'VALIDATION_F... wire wire_1.' == 'Wire clearan... wire wire_1.'
E                 
E                 - Wire clearance violation detected for wire wire_1.
E                 + VALIDATION_FAILED:Wire clearance violation detected for wire wire_1.
E                 ? ++++++++++++++++++

tests/worker_heavy/test_wire_clearance.py:79: AssertionError
------------------------------------------------------ Captured log call -------------------------------------------------------
INFO     build123d:build_common.py:345 None context requested by Box
INFO     build123d:build_common.py:345 None context requested by None
INFO     build123d:build_common.py:345 None context requested by Polyline
INFO     shared.wire_utils:wire_utils.py:215 Wire clearance violation: 0.26mm at Vector(-4.7368421052632, 0, 0)
___________________________________________________ test_prerender_24_views ____________________________________________________

mock_get_backend = <MagicMock name='get_physics_backend' id='124953219097392'>
mock_get_builder = <MagicMock name='get_simulation_builder' id='124955368741952'>
tmp_path = PosixPath('/tmp/pytest-of-maksym/pytest-2/test_prerender_24_views0')

    @patch("worker_heavy.utils.rendering.get_simulation_builder")
    @patch("worker_heavy.simulation.factory.get_physics_backend")
    def test_prerender_24_views(mock_get_backend, mock_get_builder, tmp_path):
        from worker_heavy.utils.rendering import prerender_24_views
    
        # Setup
        mock_builder = MagicMock()
        mock_get_builder.return_value = mock_builder
        mock_builder.build_from_assembly.return_value = tmp_path / "scene.xml"
        (tmp_path / "scene.xml").write_text("<mujoco/>")
    
        mock_backend = MagicMock()
        mock_get_backend.return_value = mock_backend
        # mock_backend.render_camera should return a numpy array (image)
        import numpy as np
    
        mock_backend.render_camera.return_value = np.zeros((480, 640, 3), dtype=np.uint8)
    
        with BuildPart() as p:
            Box(10, 10, 10)
        mock_component = p.part
    
        # Execute
        renders = prerender_24_views(mock_component, output_dir=str(tmp_path))
    
        # Assert
>       assert len(renders) == 24
E       AssertionError: assert 1 == 24
E        +  where 1 = len(['/tmp/pytest-of-maksym/pytest-2/test_prerender_24_views0/render_e45_a45.png'])

tests/worker_heavy/test_worker_utils.py:145: AssertionError
------------------------------------------------------ Captured log call -------------------------------------------------------
INFO     build123d:build_common.py:345 None context requested by None
INFO     build123d:build_common.py:279 Entering BuildPart with mode=Mode.ADD which is in different scope as parent
INFO     build123d:build_common.py:1275 WorkplaneList is pushing 1 workplanes: [Plane(o=(0.00, 0.00, 0.00), x=(1.00, 0.00, 0.00), z=(0.00, 0.00, 1.00))]
INFO     build123d:build_common.py:887 LocationList is pushing 1 points: [(p=(0.00, 0.00, 0.00), o=(-0.00, 0.00, -0.00))]
INFO     build123d:build_common.py:345 BuildPart context requested by Box
INFO     build123d:build_common.py:491 Completed integrating 1 object(s) into part with Mode=Mode.ADD
INFO     build123d:build_common.py:898 LocationList is popping 1 points
INFO     build123d:build_common.py:1288 WorkplaneList is popping 1 workplanes
INFO     build123d:build_common.py:321 Exiting BuildPart
________________________________________________ test_worker_client_upload_file ________________________________________________

self = <Coroutine test_worker_client_upload_file>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a4f0265370>
coro = <coroutine object test_worker_client_upload_file at 0x71a527f65e80>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
_____________________________________________ test_worker_client_read_file_binary ______________________________________________

self = <Coroutine test_worker_client_read_file_binary>

    def runtest(self) -> None:
        runner_fixture_id = f"_{self._loop_scope}_scoped_runner"
        runner = self._request.getfixturevalue(runner_fixture_id)
        context = contextvars.copy_context()
        synchronized_obj = _synchronize_coroutine(
            getattr(*self._synchronization_target_attr), runner, context
        )
        with MonkeyPatch.context() as c:
            c.setattr(*self._synchronization_target_attr, synchronized_obj)
>           super().runtest()

.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:469: 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:716: in inner
    runner.run(coro, context=context)
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 

self = <asyncio.runners.Runner object at 0x71a590516de0>
coro = <coroutine object test_worker_client_read_file_binary at 0x71a4fedd8720>

    def run(self, coro, *, context=None):
        """Run a coroutine inside the embedded event loop."""
        if not coroutines.iscoroutine(coro):
            raise ValueError("a coroutine was expected, got {!r}".format(coro))
    
        if events._get_running_loop() is not None:
            # fail fast with short traceback
>           raise RuntimeError(
                "Runner.run() cannot be called from a running event loop")
E           RuntimeError: Runner.run() cannot be called from a running event loop

/usr/lib/python3.12/asyncio/runners.py:93: RuntimeError
======================================================= warnings summary =======================================================
tests/integration/architecture_p1/test_api_fuzzing.py:60
  /home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/integration/architecture_p1/test_api_fuzzing.py:60: PytestUnknownMarkWarning: Unknown pytest.mark.worker_heavy_fuzz - is this a typo?  You can register custom marks to avoid this warning - for details, see https://docs.pytest.org/en/stable/how-to/mark.html
    @pytest.mark.worker_heavy_fuzz

tests/agent/test_benchmark_error_handling.py::test_run_generation_session_exception_handling
  /home/maksym/Work/proj/Problemologist/Problemologist-AI/controller/agent/benchmark/graph.py:280: RuntimeWarning: coroutine 'AsyncMockMixin._execute_mock_call' was never awaited
    db.add(episode)
  Enable tracemalloc to get traceback where the object was allocated.
  See https://docs.pytest.org/en/stable/how-to/capture-warnings.html#resource-warnings for more info.

tests/controller/test_steering_memory.py::test_set_and_get_user_preference
tests/controller/test_steering_memory.py::test_set_and_get_user_preference
tests/controller/test_steering_memory.py::test_get_multiple_preferences
tests/controller/test_steering_memory.py::test_get_multiple_preferences
tests/controller/test_tracing_integration.py::test_record_worker_events
tests/controller/test_tracing_integration.py::test_record_worker_events
tests/controller/test_tracing_integration.py::test_record_worker_events
tests/controller/test_tracing_integration.py::test_record_worker_events
  /home/maksym/Work/proj/Problemologist/Problemologist-AI/.venv/lib/python3.12/site-packages/sqlalchemy/sql/schema.py:3806: DeprecationWarning: datetime.datetime.utcnow() is deprecated and scheduled for removal in a future version. Use timezone-aware objects to represent datetimes in UTC: datetime.datetime.now(datetime.UTC).
    return util.wrap_callable(lambda ctx: fn(), fn)  # type: ignore

tests/electronics/test_integration_electronics.py: 413 warnings
tests/integration/test_physics_parity.py: 88 warnings
tests/worker_heavy/simulation/test_loop.py: 351 warnings
tests/worker_heavy/test_model_integration.py: 120 warnings
  /home/maksym/Work/proj/Problemologist/Problemologist-AI/.venv/lib/python3.12/site-packages/genesis/options/options.py:24: PydanticDeprecatedSince211: Accessing the 'model_fields' attribute on the instance is deprecated. Instead, you should access this attribute from the model class. Deprecated in Pydantic V2.11 to be removed in V3.0.
    allowed_params = self.model_fields.keys()

tests/electronics/test_integration_electronics.py: 49 warnings
tests/integration/test_physics_parity.py: 7 warnings
tests/worker_heavy/simulation/test_loop.py: 21 warnings
tests/worker_heavy/test_model_integration.py: 14 warnings
  /home/maksym/Work/proj/Problemologist/Problemologist-AI/.venv/lib/python3.12/site-packages/genesis/options/options.py:37: PydanticDeprecatedSince211: Accessing the 'model_fields' attribute on the instance is deprecated. Instead, you should access this attribute from the model class. Deprecated in Pydantic V2.11 to be removed in V3.0.
    for field in options.model_fields:

tests/electronics/test_integration_electronics.py: 294 warnings
tests/integration/test_physics_parity.py: 42 warnings
tests/worker_heavy/simulation/test_loop.py: 126 warnings
tests/worker_heavy/test_model_integration.py: 84 warnings
  /home/maksym/Work/proj/Problemologist/Problemologist-AI/.venv/lib/python3.12/site-packages/genesis/options/options.py:38: PydanticDeprecatedSince211: Accessing the 'model_fields' attribute on the instance is deprecated. Instead, you should access this attribute from the model class. Deprecated in Pydantic V2.11 to be removed in V3.0.
    if field in self.model_fields:

tests/electronics/test_integration_electronics.py: 7 warnings
tests/integration/test_physics_parity.py: 1 warning
tests/worker_heavy/simulation/test_loop.py: 3 warnings
  /home/maksym/Work/proj/Problemologist/Problemologist-AI/.venv/lib/python3.12/site-packages/trimesh/triangles.py:302: RuntimeWarning: invalid value encountered in divide
    center_mass = integrated[1:4] / volume

tests/generators/test_benchmark_storage.py: 1 warning
tests/generators/test_coder.py: 2 warnings
tests/generators/test_coder_objectives.py: 1 warning
tests/generators/test_graph.py: 3 warnings
tests/generators/test_planner_prompt.py: 2 warnings
tests/integration/architecture_p0/test_architecture_p0.py: 8 warnings
tests/integration/architecture_p0/test_cots_reviewer.py: 3 warnings
tests/integration/architecture_p0/test_int_026_030.py: 5 warnings
tests/integration/architecture_p0/test_int_061_063.py: 3 warnings
tests/integration/architecture_p0/test_int_102_111.py: 5 warnings
tests/integration/architecture_p0/test_int_108_meshing.py: 1 warning
tests/integration/architecture_p0/test_int_110_gpu_oom.py: 1 warning
tests/integration/architecture_p0/test_int_120_electronics.py: 5 warnings
tests/integration/architecture_p0/test_missing_p0.py: 5 warnings
tests/integration/architecture_p0/test_observability.py: 4 warnings
tests/integration/architecture_p0/test_physics_fluids.py: 4 warnings
tests/integration/architecture_p0/test_planner_gates.py: 10 warnings
tests/integration/architecture_p1/test_batch_execution.py: 1 warning
tests/integration/architecture_p1/test_benchmark_workflow.py: 1 warning
tests/integration/architecture_p1/test_engineering_loop.py: 1 warning
tests/integration/architecture_p1/test_handover.py: 1 warning
tests/integration/architecture_p1/test_infrastructure.py: 6 warnings
tests/integration/architecture_p1/test_int_064_to_069.py: 5 warnings
tests/integration/architecture_p1/test_manufacturing.py: 1 warning
tests/integration/architecture_p1/test_observability.py: 2 warnings
tests/integration/architecture_p1/test_observability_extended.py: 2 warnings
tests/integration/architecture_p1/test_physics_fluids_extended.py: 2 warnings
tests/integration/architecture_p1/test_reviewer_evidence.py: 1 warning
tests/integration/architecture_p1/test_skills_sync.py: 1 warning
tests/integration/evals_p2/test_evals_p2.py: 7 warnings
tests/integration/test_electronics_engineer.py: 2 warnings
tests/integration/test_full_workflow.py: 1 warning
tests/integration/test_multitenancy_repro.py: 1 warning
tests/integration/test_simulation_concurrency.py: 1 warning
tests/integration/test_worker_concurrency.py: 1 warning
tests/observability/test_feedback.py: 2 warnings
tests/observability/test_langfuse.py: 1 warning
tests/observability/test_persistence.py: 1 warning
tests/observability/test_storage.py: 1 warning
tests/observability/test_tracing_interaction.py: 2 warnings
tests/test_controller_persistence.py: 2 warnings
tests/test_controller_tasks.py: 2 warnings
tests/test_integration_docker.py: 2 warnings
tests/test_interrupt.py: 2 warnings
tests/test_persistence_models.py: 1 warning
tests/test_streaming_assets.py: 4 warnings
tests/test_temporal_simulation.py: 1 warning
tests/worker_light/test_binary_transfer.py: 2 warnings
  /home/maksym/Work/proj/Problemologist/Problemologist-AI/.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py:819: RuntimeWarning: An exception occurred during teardown of an asyncio.Runner. The reason is likely that you closed the underlying event loop in a test, which prevents the cleanup of asynchronous generators by the runner.
  This warning will become an error in future versions of pytest-asyncio. Please ensure that your tests don't close the event loop. Here is the traceback of the exception triggered during teardown:
  Traceback (most recent call last):
    File "/home/maksym/Work/proj/Problemologist/Problemologist-AI/.venv/lib/python3.12/site-packages/pytest_asyncio/plugin.py", line 817, in _scoped_runner
      runner.__exit__(None, None, None)
    File "/usr/lib/python3.12/asyncio/runners.py", line 62, in __exit__
      self.close()
    File "/usr/lib/python3.12/asyncio/runners.py", line 71, in close
      loop.run_until_complete(loop.shutdown_asyncgens())
    File "/usr/lib/python3.12/asyncio/base_events.py", line 663, in run_until_complete
      self._check_running()
    File "/usr/lib/python3.12/asyncio/base_events.py", line 624, in _check_running
      raise RuntimeError(
  RuntimeError: Cannot run the event loop while another loop is running
  
  
    warnings.warn(

tests/integration/architecture_p1/test_api_fuzzing.py::test_api_fuzzing[POST /episodes/{episode_id}/review]
  /usr/lib/python3.12/json/encoder.py:260: RuntimeWarning: coroutine 'test_coder_node_with_feedback' was never awaited
    def _make_iterencode(markers, _default, _encoder, _indent, _floatstr,
  Enable tracemalloc to get traceback where the object was allocated.
  See https://docs.pytest.org/en/stable/how-to/capture-warnings.html#resource-warnings for more info.

tests/integration/architecture_p1/test_api_fuzzing.py::test_api_fuzzing[POST /episodes/{episode_id}/review]
  /usr/lib/python3.12/json/encoder.py:260: RuntimeWarning: coroutine 'test_coder_node_prompt_construction' was never awaited
    def _make_iterencode(markers, _default, _encoder, _indent, _floatstr,
  Enable tracemalloc to get traceback where the object was allocated.
  See https://docs.pytest.org/en/stable/how-to/capture-warnings.html#resource-warnings for more info.

tests/integration/architecture_p1/test_api_fuzzing.py::test_api_fuzzing[POST /episodes/{episode_id}/review]
  /usr/lib/python3.12/json/encoder.py:260: RuntimeWarning: coroutine 'test_int_021_runtime_randomization_robustness' was never awaited
    def _make_iterencode(markers, _default, _encoder, _indent, _floatstr,
  Enable tracemalloc to get traceback where the object was allocated.
  See https://docs.pytest.org/en/stable/how-to/capture-warnings.html#resource-warnings for more info.

tests/integration/architecture_p1/test_api_fuzzing.py::test_api_fuzzing[POST /episodes/{episode_id}/review]
  /usr/lib/python3.12/json/encoder.py:260: RuntimeWarning: coroutine 'test_int_026_mandatory_event_families' was never awaited
    def _make_iterencode(markers, _default, _encoder, _indent, _floatstr,
  Enable tracemalloc to get traceback where the object was allocated.
  See https://docs.pytest.org/en/stable/how-to/capture-warnings.html#resource-warnings for more info.

tests/integration/architecture_p1/test_api_fuzzing.py::test_api_fuzzing[POST /episodes/{episode_id}/review]
  /usr/lib/python3.12/json/encoder.py:260: RuntimeWarning: coroutine 'test_int_062_worker_openapi_contract' was never awaited
    def _make_iterencode(markers, _default, _encoder, _indent, _floatstr,
  Enable tracemalloc to get traceback where the object was allocated.
  See https://docs.pytest.org/en/stable/how-to/capture-warnings.html#resource-warnings for more info.

tests/integration/architecture_p1/test_api_fuzzing.py::test_api_fuzzing[POST /episodes/{episode_id}/review]
  /usr/lib/python3.12/json/encoder.py:260: RuntimeWarning: coroutine 'test_int_109_physics_instability_abort' was never awaited
    def _make_iterencode(markers, _default, _encoder, _indent, _floatstr,
  Enable tracemalloc to get traceback where the object was allocated.
  See https://docs.pytest.org/en/stable/how-to/capture-warnings.html#resource-warnings for more info.

tests/integration/architecture_p1/test_api_fuzzing.py::test_api_fuzzing[POST /episodes/{episode_id}/review]
  /usr/lib/python3.12/json/encoder.py:260: RuntimeWarning: coroutine 'test_int_123_overcurrent_wire_detection' was never awaited
    def _make_iterencode(markers, _default, _encoder, _indent, _floatstr,
  Enable tracemalloc to get traceback where the object was allocated.
  See https://docs.pytest.org/en/stable/how-to/capture-warnings.html#resource-warnings for more info.

tests/integration/architecture_p1/test_api_fuzzing.py::test_api_fuzzing[POST /episodes/{episode_id}/review]
  /usr/lib/python3.12/json/encoder.py:260: RuntimeWarning: coroutine 'test_int_025_events_collection_e2e' was never awaited
    def _make_iterencode(markers, _default, _encoder, _indent, _floatstr,
  Enable tracemalloc to get traceback where the object was allocated.
  See https://docs.pytest.org/en/stable/how-to/capture-warnings.html#resource-warnings for more info.

tests/integration/architecture_p1/test_api_fuzzing.py::test_api_fuzzing[POST /episodes/{episode_id}/review]
  /usr/lib/python3.12/json/encoder.py:260: RuntimeWarning: coroutine 'test_int_105_fluid_containment_evaluation' was never awaited
    def _make_iterencode(markers, _default, _encoder, _indent, _floatstr,
  Enable tracemalloc to get traceback where the object was allocated.
  See https://docs.pytest.org/en/stable/how-to/capture-warnings.html#resource-warnings for more info.

tests/integration/architecture_p1/test_api_fuzzing.py::test_api_fuzzing[POST /episodes/{episode_id}/review]
  /usr/lib/python3.12/json/encoder.py:260: RuntimeWarning: coroutine 'test_int_008_objectives_validation' was never awaited
    def _make_iterencode(markers, _default, _encoder, _indent, _floatstr,
  Enable tracemalloc to get traceback where the object was allocated.
  See https://docs.pytest.org/en/stable/how-to/capture-warnings.html#resource-warnings for more info.

tests/integration/architecture_p1/test_api_fuzzing.py::test_api_fuzzing[POST /episodes/{episode_id}/review]
  /usr/lib/python3.12/json/encoder.py:260: RuntimeWarning: coroutine 'test_int_018_validate_and_price_integration_gate' was never awaited
    def _make_iterencode(markers, _default, _encoder, _indent, _floatstr,
  Enable tracemalloc to get traceback where the object was allocated.
  See https://docs.pytest.org/en/stable/how-to/capture-warnings.html#resource-warnings for more info.

tests/integration/test_fem_breakage.py::test_fem_breakage_detection
  /usr/lib/python3.12/unittest/mock.py:2188: RuntimeWarning: coroutine 'test_journal_quality_integration_int_051' was never awaited
    def __init__(self, name, parent):
  Enable tracemalloc to get traceback where the object was allocated.
  See https://docs.pytest.org/en/stable/how-to/capture-warnings.html#resource-warnings for more info.

tests/integration/test_fem_breakage.py::test_fem_breakage_detection
  /usr/lib/python3.12/unittest/mock.py:2188: RuntimeWarning: coroutine 'test_skill_effectiveness_tracking_int_052' was never awaited
    def __init__(self, name, parent):
  Enable tracemalloc to get traceback where the object was allocated.
  See https://docs.pytest.org/en/stable/how-to/capture-warnings.html#resource-warnings for more info.

tests/integration/test_fem_breakage.py::test_fem_breakage_detection
  /usr/lib/python3.12/unittest/mock.py:2188: RuntimeWarning: coroutine 'test_electromechanical_handoff' was never awaited
    def __init__(self, name, parent):
  Enable tracemalloc to get traceback where the object was allocated.
  See https://docs.pytest.org/en/stable/how-to/capture-warnings.html#resource-warnings for more info.

tests/integration/test_fem_breakage.py::test_fem_breakage_detection
  /usr/lib/python3.12/unittest/mock.py:2188: RuntimeWarning: coroutine 'test_electronics_engineer_passthrough' was never awaited
    def __init__(self, name, parent):
  Enable tracemalloc to get traceback where the object was allocated.
  See https://docs.pytest.org/en/stable/how-to/capture-warnings.html#resource-warnings for more info.

tests/integration/test_genesis_builder.py::test_genesis_builder_generates_msh_when_fem_enabled
  /home/maksym/Work/proj/Problemologist/Problemologist-AI/.venv/lib/python3.12/site-packages/trimesh/exchange/gltf/__init__.py:554: RuntimeWarning: coroutine 'test_full_workflow_end_to_end' was never awaited
    blob["bufferView"] = list(buff.keys()).index(hashed)
  Enable tracemalloc to get traceback where the object was allocated.
  See https://docs.pytest.org/en/stable/how-to/capture-warnings.html#resource-warnings for more info.

tests/integration/test_physics_fluids_wp04.py::test_flow_rate_integration
  /usr/lib/python3.12/unittest/mock.py:2129: RuntimeWarning: coroutine 'test_multitenancy_repro' was never awaited
    if getattr(self, "_mock_methods", None) is not None:
  Enable tracemalloc to get traceback where the object was allocated.
  See https://docs.pytest.org/en/stable/how-to/capture-warnings.html#resource-warnings for more info.

tests/integration/test_physics_genesis.py::test_fluid_containment_integration
tests/integration/test_physics_genesis.py::test_fluid_containment_integration
tests/integration/test_physics_genesis.py::test_fluid_containment_failure
tests/integration/test_physics_genesis.py::test_fluid_containment_failure
  /home/maksym/Work/proj/Problemologist/Problemologist-AI/.venv/lib/python3.12/site-packages/pydantic/main.py:250: DeprecationWarning: In future, it will be an error for 'np.bool' scalars to be interpreted as an index
    validated_self = self.__pydantic_validator__.validate_python(data, self_instance=self)

tests/observability/test_storage.py::test_upload_download
  /home/maksym/Work/proj/Problemologist/Problemologist-AI/.venv/lib/python3.12/site-packages/responses/__init__.py:384: RuntimeWarning: coroutine 'test_report_trace_feedback_success' was never awaited
    def __init__(
  Enable tracemalloc to get traceback where the object was allocated.
  See https://docs.pytest.org/en/stable/how-to/capture-warnings.html#resource-warnings for more info.

tests/observability/test_storage.py::test_upload_download
  /home/maksym/Work/proj/Problemologist/Problemologist-AI/.venv/lib/python3.12/site-packages/responses/__init__.py:384: RuntimeWarning: coroutine 'test_report_trace_feedback_no_langfuse_id' was never awaited
    def __init__(
  Enable tracemalloc to get traceback where the object was allocated.
  See https://docs.pytest.org/en/stable/how-to/capture-warnings.html#resource-warnings for more info.

tests/observability/test_storage.py::test_upload_download
  /home/maksym/Work/proj/Problemologist/Problemologist-AI/.venv/lib/python3.12/site-packages/responses/__init__.py:384: RuntimeWarning: coroutine 'test_calculate_and_report_automated_score' was never awaited
    def __init__(
  Enable tracemalloc to get traceback where the object was allocated.
  See https://docs.pytest.org/en/stable/how-to/capture-warnings.html#resource-warnings for more info.

tests/test_api_fuzzing.py::test_api_fuzzing[POST /fs/ls]
  /home/maksym/Work/proj/Problemologist/Problemologist-AI/.venv/lib/python3.12/site-packages/hypothesis/strategies/_internal/lazy.py:139: RuntimeWarning: coroutine 'test_reviewer_evidence_completeness' was never awaited
    def map(self, pack):
  Enable tracemalloc to get traceback where the object was allocated.
  See https://docs.pytest.org/en/stable/how-to/capture-warnings.html#resource-warnings for more info.

tests/test_api_fuzzing.py::test_api_fuzzing[POST /fs/ls]
  /home/maksym/Work/proj/Problemologist/Problemologist-AI/.venv/lib/python3.12/site-packages/hypothesis/strategies/_internal/lazy.py:139: RuntimeWarning: coroutine 'test_dataset_readiness_completeness_int_050' was never awaited
    def map(self, pack):
  Enable tracemalloc to get traceback where the object was allocated.
  See https://docs.pytest.org/en/stable/how-to/capture-warnings.html#resource-warnings for more info.

tests/test_api_fuzzing.py::test_api_fuzzing[POST /fs/ls]
  /home/maksym/Work/proj/Problemologist/Problemologist-AI/.venv/lib/python3.12/site-packages/hypothesis/strategies/_internal/lazy.py:139: RuntimeWarning: coroutine 'test_worker_concurrency' was never awaited
    def map(self, pack):
  Enable tracemalloc to get traceback where the object was allocated.
  See https://docs.pytest.org/en/stable/how-to/capture-warnings.html#resource-warnings for more info.

tests/test_api_fuzzing.py::test_api_fuzzing[POST /fs/ls]
  /home/maksym/Work/proj/Problemologist/Problemologist-AI/.venv/lib/python3.12/site-packages/hypothesis/strategies/_internal/lazy.py:139: RuntimeWarning: coroutine 'test_setup_persistence' was never awaited
    def map(self, pack):
  Enable tracemalloc to get traceback where the object was allocated.
  See https://docs.pytest.org/en/stable/how-to/capture-warnings.html#resource-warnings for more info.

tests/test_api_fuzzing.py::test_api_fuzzing[POST /fs/ls]
  /home/maksym/Work/proj/Problemologist/Problemologist-AI/.venv/lib/python3.12/site-packages/hypothesis/strategies/_internal/lazy.py:139: RuntimeWarning: coroutine 'test_async_upload_download' was never awaited
    def map(self, pack):
  Enable tracemalloc to get traceback where the object was allocated.
  See https://docs.pytest.org/en/stable/how-to/capture-warnings.html#resource-warnings for more info.

tests/test_api_fuzzing.py::test_api_fuzzing[POST /fs/ls]
  /home/maksym/Work/proj/Problemologist/Problemologist-AI/.venv/lib/python3.12/site-packages/hypothesis/strategies/_internal/lazy.py:139: RuntimeWarning: coroutine 'test_graph_initializes_langfuse_callback' was never awaited
    def map(self, pack):
  Enable tracemalloc to get traceback where the object was allocated.
  See https://docs.pytest.org/en/stable/how-to/capture-warnings.html#resource-warnings for more info.

tests/test_episodes_api.py::test_get_episode_success
tests/test_episodes_api.py::test_list_episodes
  /home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/test_episodes_api.py:30: DeprecationWarning: datetime.datetime.utcnow() is deprecated and scheduled for removal in a future version. Use timezone-aware objects to represent datetimes in UTC: datetime.datetime.now(datetime.UTC).
    mock_episode.created_at = datetime.utcnow()

tests/test_episodes_api.py::test_get_episode_success
tests/test_episodes_api.py::test_list_episodes
  /home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/test_episodes_api.py:31: DeprecationWarning: datetime.datetime.utcnow() is deprecated and scheduled for removal in a future version. Use timezone-aware objects to represent datetimes in UTC: datetime.datetime.now(datetime.UTC).
    mock_episode.updated_at = datetime.utcnow()

tests/test_episodes_api.py::test_get_episode_success
  /home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/test_episodes_api.py:64: DeprecationWarning: datetime.datetime.utcnow() is deprecated and scheduled for removal in a future version. Use timezone-aware objects to represent datetimes in UTC: datetime.datetime.now(datetime.UTC).
    mock_asset.created_at = datetime.utcnow()

tests/unit/test_validation_utils.py::test_set_soft_mesh
tests/worker_heavy/utils/test_physics_utils.py::test_set_soft_mesh
  /home/maksym/Work/proj/Problemologist/Problemologist-AI/.venv/lib/python3.12/site-packages/pydantic/main.py:464: UserWarning: Pydantic serializer warnings:
    PydanticSerializationUnexpectedValue(Expected `enum` - serialized value may not be as expected [field_name='backend', input_value='genesis', input_type=str])
    return self.__pydantic_serializer__.to_python(

tests/workbenches/test_cnc.py::test_cnc_undercut_part
  /home/maksym/Work/proj/Problemologist/Problemologist-AI/.venv/lib/python3.12/site-packages/build123d/topology/shape_core.py:2588: RuntimeWarning: coroutine 'test_get_asset_py' was never awaited
    if callable(filter_by):
  Enable tracemalloc to get traceback where the object was allocated.
  See https://docs.pytest.org/en/stable/how-to/capture-warnings.html#resource-warnings for more info.

tests/workbenches/test_cnc.py::test_cnc_undercut_part
  /home/maksym/Work/proj/Problemologist/Problemologist-AI/.venv/lib/python3.12/site-packages/build123d/topology/shape_core.py:2588: RuntimeWarning: coroutine 'test_get_asset_syntax_error' was never awaited
    if callable(filter_by):
  Enable tracemalloc to get traceback where the object was allocated.
  See https://docs.pytest.org/en/stable/how-to/capture-warnings.html#resource-warnings for more info.

tests/workbenches/test_cnc.py::test_cnc_undercut_part
  /home/maksym/Work/proj/Problemologist/Problemologist-AI/.venv/lib/python3.12/site-packages/build123d/topology/shape_core.py:2588: RuntimeWarning: coroutine 'test_remote_fs_middleware_temporal' was never awaited
    if callable(filter_by):
  Enable tracemalloc to get traceback where the object was allocated.
  See https://docs.pytest.org/en/stable/how-to/capture-warnings.html#resource-warnings for more info.

-- Docs: https://docs.pytest.org/en/stable/how-to/capture-warnings.html
=================================================== short test summary info ====================================================
FAILED tests/controller/agent/test_refactor_verification.py::test_all_agents_initialization - AttributeError: <module 'controller.agent.benchmark.nodes' from '/home/maksym/Work/proj/Problemologist/Problemologist-AI/co...
FAILED tests/controller/agent/test_reward_generic.py::test_metric_benchmark_planner_basic - AssertionError: assert Prediction(\n ...sult: False'\n) == 0.45 ± 4.5e-07
FAILED tests/controller/agent/test_reward_generic.py::test_metric_benchmark_planner_cost_overage - AssertionError: assert Prediction(\n ...sult: False'\n) == 0.23000000000000004 ± 2.3e-07
FAILED tests/controller/agent/test_reward_generic.py::test_metric_cad_engineer_failure_formula - AssertionError: assert Prediction(\n ...ress: 0.12)'\n) == 0.472 ± 4.7e-07
FAILED tests/controller/agent/test_reward_generic.py::test_metric_reviewer_generic_binary - AssertionError: assert Prediction(\n ...esult: True'\n) == 1.0 ± 1.0e-06
FAILED tests/controller/agent/test_reward_metric.py::test_metric_full_success - AssertionError: assert 0.85 == 1.0
FAILED tests/controller/agent/test_reward_metric.py::test_metric_partial_sim - AssertionError: assert 0.37 == 0.52
FAILED tests/controller/agent/test_reward_metric.py::test_metric_cost_overage - assert 0.85 == 0.95 ± 9.5e-07
FAILED tests/controller/test_remote_fs_events.py::test_simulate_emits_events - AttributeError: <module 'controller.middleware.remote_fs' from '/home/maksym/Work/proj/Problemologist/Problemologist-AI/con...
FAILED tests/controller/test_remote_fs_events.py::test_validate_emits_events - AttributeError: <module 'controller.middleware.remote_fs' from '/home/maksym/Work/proj/Problemologist/Problemologist-AI/con...
FAILED tests/controller/test_remote_fs_events.py::test_submit_emits_events - AttributeError: <module 'controller.middleware.remote_fs' from '/home/maksym/Work/proj/Problemologist/Problemologist-AI/con...
FAILED tests/controller/test_remote_fs_events.py::test_ls_files_emits_event - AttributeError: <module 'controller.middleware.remote_fs' from '/home/maksym/Work/proj/Problemologist/Problemologist-AI/con...
FAILED tests/controller/test_remote_fs_events.py::test_read_file_emits_events - AttributeError: <module 'controller.middleware.remote_fs' from '/home/maksym/Work/proj/Problemologist/Problemologist-AI/con...
FAILED tests/controller/test_remote_fs_events.py::test_write_file_emits_event - AttributeError: <module 'controller.middleware.remote_fs' from '/home/maksym/Work/proj/Problemologist/Problemologist-AI/con...
FAILED tests/controller/test_remote_fs_events.py::test_edit_file_emits_event - AttributeError: <module 'controller.middleware.remote_fs' from '/home/maksym/Work/proj/Problemologist/Problemologist-AI/con...
FAILED tests/controller/test_remote_fs_events.py::test_grep_emits_event - AttributeError: <module 'controller.middleware.remote_fs' from '/home/maksym/Work/proj/Problemologist/Problemologist-AI/con...
FAILED tests/controller/test_remote_fs_events.py::test_run_command_emits_event - AttributeError: <module 'controller.middleware.remote_fs' from '/home/maksym/Work/proj/Problemologist/Problemologist-AI/con...
FAILED tests/cots/test_cots_events.py::test_cots_search_event - pydantic_core._pydantic_core.ValidationError: 3 validation errors for COTSSearchEvent
FAILED tests/cots/test_indexer.py::test_indexer_basic - assert 10 == 9
FAILED tests/cots/test_search_agent.py::test_search_cots_catalog_tool - AttributeError: 'function' object has no attribute 'invoke'
FAILED tests/cots/test_search_agent.py::test_search_cots_catalog_no_results - AttributeError: 'function' object has no attribute 'invoke'
FAILED tests/cots/test_search_agent.py::test_create_cots_search_agent - AttributeError: <module 'shared.cots.agent' from '/home/maksym/Work/proj/Problemologist/Problemologist-AI/shared/cots/agent...
FAILED tests/e2e/test_playwright_benchmark.py::test_benchmark_creation_flow[chromium] - playwright._impl._errors.Error: Page.goto: net::ERR_CONNECTION_REFUSED at http://localhost:5173/
FAILED tests/electronics/test_integration_electronics.py::test_int_126_wire_tear_failure - AssertionError: assert 'wire_torn:wire_torn_test' in 'WIRE_TORN:wire_torn_test'
FAILED tests/electronics/test_integration_electronics.py::test_int_120_circuit_validation_pre_gate - assert ('validation_failed' in "VALIDATION_FAILED:test_int_120_circuit_validation_pre_gate.<locals>.mock_validate() got an ...
FAILED tests/electronics/test_integration_electronics.py::test_int_121_short_circuit_detection - assert 'FAILED_SHORT_CIRCUIT' in "VALIDATION_FAILED:test_int_121_short_circuit_detection.<locals>.mock_validate() got an un...
FAILED tests/electronics/test_integration_electronics.py::test_int_122_overcurrent_supply_detection - assert 'FAILED_OVERCURRENT_SUPPLY' in "VALIDATION_FAILED:test_int_122_overcurrent_supply_detection.<locals>.mock_validate()...
FAILED tests/electronics/test_integration_electronics.py::test_int_124_open_circuit_detection - assert 'FAILED_OPEN_CIRCUIT' in "VALIDATION_FAILED:test_int_124_open_circuit_detection.<locals>.mock_validate() got an unex...
FAILED tests/generators/test_coder.py::test_coder_node_success - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/generators/test_coder.py::test_coder_node_with_feedback - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/generators/test_coder_objectives.py::test_coder_node_injects_objectives_yaml - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/generators/test_graph.py::test_define_graph_compiles - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/generators/test_graph.py::test_run_generation_session_mocked - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/generators/test_graph.py::test_run_generation_session_rejected - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/generators/test_planner_prompt.py::test_planner_node_prompt_construction - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/generators/test_planner_prompt.py::test_coder_node_prompt_construction - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_architecture_p0.py::test_int_001_compose_boot_health_contract - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_architecture_p0.py::test_int_002_controller_worker_execution_boundary - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_architecture_p0.py::test_int_003_session_filesystem_isolation - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_architecture_p0.py::test_int_004_simulation_serialization - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_architecture_p0.py::test_int_020_simulation_failure_taxonomy - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_architecture_p0.py::test_int_021_runtime_randomization_robustness - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_architecture_p0.py::test_int_022_motor_overload_behavior - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_architecture_p0.py::test_int_023_fastener_validity_rules - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_cots_reviewer.py::test_int_012_013_cots_search_contract_and_readonly - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_cots_reviewer.py::test_int_016_reviewer_decision_schema_gate - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_cots_reviewer.py::test_int_017_plan_refusal_loop - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_int_026_030.py::test_int_026_mandatory_event_families - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_int_026_030.py::test_int_027_seed_variant_tracking - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_int_026_030.py::test_int_028_strict_api_schema_contract - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_int_026_030.py::test_int_029_api_key_enforcement - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_int_026_030.py::test_int_030_interrupt_propagation - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_int_061_063.py::test_int_061_asset_serving_security - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_int_061_063.py::test_int_062_worker_openapi_contract - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_int_061_063.py::test_int_063_mounted_path_read_only - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_int_102_111.py::test_int_102_111_fem_material_validation - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_int_102_111.py::test_int_103_part_breakage_detection - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_int_102_111.py::test_int_104_stress_reporting - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_int_102_111.py::test_int_107_stress_objective_evaluation - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_int_102_111.py::test_int_109_physics_instability_abort - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_int_108_meshing.py::test_int_108_tetrahedralization_pipeline - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_int_110_gpu_oom.py::test_int_110_gpu_oom_retry - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_int_120_electronics.py::test_int_120_circuit_validation_gate - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_int_120_electronics.py::test_int_121_short_circuit_detection - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_int_120_electronics.py::test_int_122_overcurrent_supply_detection - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_int_120_electronics.py::test_int_123_overcurrent_wire_detection - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_int_120_electronics.py::test_int_124_open_circuit_detection - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_missing_p0.py::test_int_004_episode_artifact_persistence - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_missing_p0.py::test_int_005_trace_realtime_broadcast - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_missing_p0.py::test_int_011_planner_target_caps_validation - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_missing_p0.py::test_int_014_cots_propagation - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_missing_p0.py::test_int_025_events_collection_e2e - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_observability.py::test_int_053_temporal_workflow_lifecycle - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_observability.py::test_int_055_s3_artifact_upload_logging - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_observability.py::test_int_054_temporal_failure_path - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_observability.py::test_int_056_s3_upload_failure_retry - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_physics_fluids.py::test_int_101_physics_backend_selection - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_physics_fluids.py::test_int_105_fluid_containment_evaluation - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_physics_fluids.py::test_int_106_flow_rate_evaluation - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_physics_fluids.py::test_int_112_mujoco_backward_compat - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_planner_gates.py::test_int_005_mandatory_artifacts_gate - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_planner_gates.py::test_int_006_plan_structure_validation - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_planner_gates.py::test_int_007_todo_integrity - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_planner_gates.py::test_int_008_objectives_validation - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_planner_gates.py::test_int_009_cost_estimation_validation - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_planner_gates.py::test_int_011_planner_caps_enforcement - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_planner_gates.py::test_int_015_engineer_handover_immutability - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_planner_gates.py::test_int_019_hard_constraints_gates - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_planner_gates.py::test_int_010_planner_pricing_script_integration - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p0/test_planner_gates.py::test_int_018_validate_and_price_integration_gate - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p1/test_api_fuzzing.py::test_api_fuzzing[GET /cots/metadata] - + Exception Group Traceback (most recent call last):
FAILED tests/integration/architecture_p1/test_batch_execution.py::test_int_043_batch_execution_path - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p1/test_benchmark_workflow.py::test_benchmark_planner_cad_reviewer_path - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p1/test_engineering_loop.py::test_engineering_full_loop - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p1/test_handover.py::test_benchmark_to_engineer_handoff - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p1/test_infrastructure.py::test_render_artifact_generation_int_039 - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p1/test_infrastructure.py::test_asset_persistence_linkage_int_040 - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p1/test_infrastructure.py::test_mjcf_joint_mapping_int_037 - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p1/test_infrastructure.py::test_controller_function_family_int_038 - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p1/test_infrastructure.py::test_temporal_recovery_int_041 - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p1/test_infrastructure.py::test_async_callbacks_int_042 - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p1/test_int_064_to_069.py::test_int_064_cots_metadata - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p1/test_int_064_to_069.py::test_int_065_skills_safety - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p1/test_int_064_to_069.py::test_int_066_fluid_electronics_coupling - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p1/test_int_064_to_069.py::test_int_067_068_steerability - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p1/test_int_064_to_069.py::test_int_069_frontend_contract - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p1/test_manufacturing.py::test_manufacturing_methods_and_materials - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p1/test_observability.py::test_int_059_langfuse_trace_linkage - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p1/test_observability.py::test_int_060_langfuse_feedback_contract - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p1/test_observability_extended.py::test_int_057_backup_logging_flow - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p1/test_observability_extended.py::test_int_058_cross_system_correlation - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p1/test_physics_fluids_extended.py::test_int_138_smoke_test_mode - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p1/test_physics_fluids_extended.py::test_int_139_fluid_storage_policy - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p1/test_reviewer_evidence.py::test_reviewer_evidence_completeness - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/architecture_p1/test_skills_sync.py::test_int_045_skills_sync_lifecycle - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/evals_p2/test_evals_p2.py::test_plan_to_cad_fidelity_regression_int_046 - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/evals_p2/test_evals_p2.py::test_cross_seed_transfer_uplift_int_047 - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/evals_p2/test_evals_p2.py::test_reviewer_optimality_regression_int_048 - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/evals_p2/test_evals_p2.py::test_evaluation_metric_materialization_int_049 - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/evals_p2/test_evals_p2.py::test_dataset_readiness_completeness_int_050 - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/evals_p2/test_evals_p2.py::test_journal_quality_integration_int_051 - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/evals_p2/test_evals_p2.py::test_skill_effectiveness_tracking_int_052 - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/test_electronics_engineer.py::test_electromechanical_handoff - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/test_electronics_engineer.py::test_electronics_engineer_passthrough - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/test_fem_breakage.py::test_fem_breakage_detection - TypeError: '>' not supported between instances of 'MagicMock' and 'float'
FAILED tests/integration/test_fluid_containment.py::test_fluid_containment_logic - TypeError: '>' not supported between instances of 'MagicMock' and 'float'
FAILED tests/integration/test_full_workflow.py::test_full_workflow_end_to_end - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/test_genesis_builder.py::test_genesis_builder_generates_msh_when_fem_enabled - ValueError: string is not a file: `/tmp/pytest-of-maksym/pytest-2/test_genesis_builder_generates0/output/assets/test_part.r...
FAILED tests/integration/test_gpu_oom_retry.py::test_gpu_oom_retry_logic - AttributeError: <module 'worker_heavy.utils.validation' from '/home/maksym/Work/proj/Problemologist/Problemologist-AI/worke...
FAILED tests/integration/test_multitenancy_repro.py::test_multitenancy_repro - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/test_physics_fluids_wp04.py::test_flow_rate_integration - AssertionError: assert False is True
FAILED tests/integration/test_physics_genesis.py::test_fluid_containment_failure - AssertionError: assert True is False
FAILED tests/integration/test_simulation_concurrency.py::test_simulation_concurrency_serialization - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/integration/test_worker_concurrency.py::test_worker_concurrency - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/observability/test_feedback.py::test_report_trace_feedback_success - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/observability/test_feedback.py::test_report_trace_feedback_no_langfuse_id - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/observability/test_langfuse.py::test_calculate_and_report_automated_score - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/observability/test_persistence.py::test_setup_persistence - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/observability/test_storage.py::test_async_upload_download - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/observability/test_tracing_interaction.py::test_nodes_call_get_langfuse_callback - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/observability/test_tracing_interaction.py::test_graph_initializes_langfuse_callback - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/test_controller_api_extended.py::test_websocket_manager - RuntimeError: asyncio.run() cannot be called from a running event loop
FAILED tests/test_controller_api_extended.py::test_websocket_broadcast_failure - RuntimeError: asyncio.run() cannot be called from a running event loop
FAILED tests/test_controller_tasks.py::test_execute_agent_task_success - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/test_controller_tasks.py::test_execute_agent_task_without_langfuse_callback - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/test_cots_search.py::test_search_cots_catalog_success - AttributeError: 'function' object has no attribute 'invoke'
FAILED tests/test_cots_search.py::test_search_cots_catalog_no_results - AttributeError: 'function' object has no attribute 'invoke'
FAILED tests/test_cots_search.py::test_search_cots_catalog_all_args - AttributeError: 'function' object has no attribute 'invoke'
FAILED tests/test_integration_docker.py::test_services_health - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/test_integration_docker.py::test_controller_to_worker_agent_run - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/test_interrupt.py::test_execute_agent_task_cancelled - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/test_interrupt.py::test_interrupt_episode_success - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/test_persistence_models.py::test_db_setup - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/test_streaming_assets.py::test_episode_broadcaster - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/test_streaming_assets.py::test_get_asset_glb - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/test_streaming_assets.py::test_get_asset_py - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/test_streaming_assets.py::test_get_asset_syntax_error - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/test_temporal_simulation.py::test_remote_fs_middleware_temporal - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/workbenches/test_config.py::test_load_config - assert False
FAILED tests/workbenches/test_print_3d.py::test_3dp_valid_part - AssertionError: assert 'cost_breakdown' in WorkbenchMetadata(cost_breakdown=CostBreakdown(process='print_3d', total_cost=11...
FAILED tests/worker_heavy/simulation/test_dynamic_control.py::test_dynamic_controllers - assert 0.0 == 10.0
FAILED tests/worker_heavy/simulation/test_loop.py::test_metrics_collection - AssertionError: assert 0.0 >= 5.0
FAILED tests/worker_heavy/simulation/test_loop.py::test_goal_zone_trigger - AssertionError: assert False is True
FAILED tests/worker_heavy/simulation/test_loop.py::test_forbidden_zone_trigger - AssertionError: assert None == <FailureReason.FORBID_ZONE_HIT: 'FORBID_ZONE_HIT'>
FAILED tests/worker_heavy/simulation/test_loop.py::test_target_fell_off_world - AssertionError: assert None == <FailureReason.OUT_OF_BOUNDS: 'OUT_OF_BOUNDS'>
FAILED tests/worker_heavy/simulation/test_loop.py::test_instability_detection - AssertionError: assert None == <FailureReason.PHYSICS_INSTABILITY: 'PHYSICS_INSTABILITY'>
FAILED tests/worker_heavy/simulation/test_loop_stress.py::TestSimulationLoopStress::test_stress_collection - AssertionError: assert 0 >= 3
FAILED tests/worker_heavy/simulation/test_motor_overload.py::TestMotorOverload::test_overload_detection_triggers - AssertionError: assert 'motor_overload' in 'MOTOR_OVERLOAD:servo'
FAILED tests/worker_heavy/test_genesis_interaction.py::test_apply_control - TypeError: cannot unpack non-iterable NoneType object
FAILED tests/worker_heavy/test_validation.py::test_simulation - ValueError: Part 'unknown' is missing required metadata. Every part must have a .metadata attribute (PartMetadata or Compou...
FAILED tests/worker_heavy/test_wire_clearance.py::test_wire_clearance_violation - AssertionError: assert 'VALIDATION_F... wire wire_1.' == 'Wire clearan... wire wire_1.'
FAILED tests/worker_heavy/test_worker_utils.py::test_prerender_24_views - AssertionError: assert 1 == 24
FAILED tests/worker_light/test_binary_transfer.py::test_worker_client_upload_file - RuntimeError: Runner.run() cannot be called from a running event loop
FAILED tests/worker_light/test_binary_transfer.py::test_worker_client_read_file_binary - RuntimeError: Runner.run() cannot be called from a running event loop
ERROR tests/controller/test_remote_fs.py::test_remote_fs_middleware_ls - AttributeError: <module 'controller.middleware.remote_fs' from '/home/maksym/Work/proj/Problemologist/Problemologist-AI/con...
ERROR tests/controller/test_remote_fs.py::test_remote_fs_middleware_write_protection - AttributeError: <module 'controller.middleware.remote_fs' from '/home/maksym/Work/proj/Problemologist/Problemologist-AI/con...
ERROR tests/controller/test_remote_fs.py::test_fs_tools_execution - AttributeError: <module 'controller.middleware.remote_fs' from '/home/maksym/Work/proj/Problemologist/Problemologist-AI/con...
ERROR tests/controller/test_remote_fs.py::test_remote_fs_middleware_read - AttributeError: <module 'controller.middleware.remote_fs' from '/home/maksym/Work/proj/Problemologist/Problemologist-AI/con...
ERROR tests/controller/test_remote_fs.py::test_remote_fs_middleware_overwrite - AttributeError: <module 'controller.middleware.remote_fs' from '/home/maksym/Work/proj/Problemologist/Problemologist-AI/con...
ERROR tests/generators/test_benchmark_storage.py::test_save_asset_with_variants - RuntimeError: Runner.run() cannot be called from a running event loop
ERROR tests/test_controller_persistence.py::test_asset_content_persistence - RuntimeError: Runner.run() cannot be called from a running event loop
ERROR tests/test_controller_persistence.py::test_episode_relationships - RuntimeError: Runner.run() cannot be called from a running event loop
=================================================== warnings summary (final) ===================================================
.venv/lib/python3.12/site-packages/_hypothesis_pytestplugin.py:393
  /home/maksym/Work/proj/Problemologist/Problemologist-AI/.venv/lib/python3.12/site-packages/_hypothesis_pytestplugin.py:393: RuntimeWarning: coroutine 'test_worker_client_read_file_binary' was never awaited
    stats = report.__dict__.get(STATS_KEY)
  Enable tracemalloc to get traceback where the object was allocated.
  See https://docs.pytest.org/en/stable/how-to/capture-warnings.html#resource-warnings for more info.

-- Docs: https://docs.pytest.org/en/stable/how-to/capture-warnings.html
============================= 174 failed, 405 passed, 1794 warnings, 8 errors in 188.61s (0:03:08) =============================
/home/maksym/Work/proj/Problemologist/Problemologist-AI/.venv/lib/python3.12/site-packages/_pytest/unraisableexception.py:33: RuntimeWarning: coroutine 'ConnectionManager.connect' was never awaited
  gc.collect()
RuntimeWarning: Enable tracemalloc to get the object allocation traceback
/home/maksym/Work/proj/Problemologist/Problemologist-AI/.venv/lib/python3.12/site-packages/_pytest/unraisableexception.py:33: RuntimeWarning: coroutine 'test_services_health' was never awaited
  gc.collect()
RuntimeWarning: Enable tracemalloc to get the object allocation traceback
/home/maksym/Work/proj/Problemologist/Problemologist-AI/.venv/lib/python3.12/site-packages/_pytest/unraisableexception.py:33: RuntimeWarning: coroutine 'test_get_asset_glb' was never awaited
  gc.collect()
RuntimeWarning: Enable tracemalloc to get the object allocation traceback
/home/maksym/Work/proj/Problemologist/Problemologist-AI/.venv/lib/python3.12/site-packages/_pytest/unraisableexception.py:33: RuntimeWarning: coroutine 'test_worker_client_upload_file' was never awaited
  gc.collect()
RuntimeWarning: Enable tracemalloc to get the object allocation traceback
[Genesis] [10:20:56] [INFO] 💤 Exiting Genesis and caching compiled kernels...
💤 Exiting Genesis and caching compiled kernels...
Integration tests FAILED!

Cleaning up processes (Controller: 29923, Worker Light: 29921, Worker Heavy: 29922, Temporal: 29924, Xvfb: 29795)...
./scripts/run_integration_tests.sh: line 206: 29923 Killed                  uv run uvicorn controller.api.main:app --host 0.0.0.0 --port 18000 > "$LOG_DIR/controller.log" 2>&1
./scripts/run_integration_tests.sh: line 206: 29921 Killed                  uv run uvicorn worker_light.app:app --host 0.0.0.0 --port 18001 > "$LOG_DIR/worker_light.log" 2>&1
./scripts/run_integration_tests.sh: line 206: 29922 Killed                  uv run uvicorn worker_heavy.app:app --host 0.0.0.0 --port 18002 > "$LOG_DIR/worker_heavy.log" 2>&1
Bringing down infrastructure containers...
[+] down 8/8
 ✔ Container problemologist-ai-temporal-ui-1   Removed                                                                      0.2s
 ✔ Container problemologist-ai-createbuckets-1 Removed                                                                      0.0s
 ✔ Container problemologist-ai-minio-1         Removed                                                                      0.2s
 ✔ Container problemologist-ai-temporal-1      Removed                                                                      1.1s
 ✔ Container problemologist-ai-postgres-1      Removed                                                                      0.2s
 ✔ Volume problemologist-ai_postgres_test_data Removed                                                                      0.0s
 ✔ Volume problemologist-ai_minio_test_data    Removed                                                                      0.0s
 ✔ Network problemologist-ai_default           Removed                                                                      0.1s
maksym@maksym-Latitude-5410:~/Work/proj/Problemologist/Problemologist-AI$ 
