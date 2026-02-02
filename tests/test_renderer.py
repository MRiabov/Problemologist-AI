from unittest.mock import MagicMock, patch

from src.generators.benchmark.renderer import render_scenario


def test_render_scenario_basic(tmp_path):
    # Mock mujoco
    mock_model = MagicMock()
    mock_model.ncam = 0
    mock_data = MagicMock()

    with (
        patch("mujoco.MjModel.from_xml_string", return_value=mock_model),
        patch("mujoco.MjData", return_value=mock_data),
        patch("mujoco.mj_forward"),
        patch("mujoco.Renderer") as mock_renderer_cls,
    ):
        mock_renderer = mock_renderer_cls.return_value
        mock_renderer.render.return_value = MagicMock()  # PIL image data mock

        output_prefix = str(tmp_path / "test")
        xml = "<mujoco/>"

        # We need to mock Image.fromarray(pixels).save(path)
        with patch(
            "src.generators.benchmark.renderer.Image.fromarray"
        ) as mock_fromarray:
            mock_img = mock_fromarray.return_value

            paths = render_scenario(xml, output_prefix)

            assert len(paths) == 1
            assert paths[0] == f"{output_prefix}_default.png"
            mock_renderer.update_scene.assert_called_once_with(mock_data)
            mock_img.save.assert_called_once()


def test_render_scenario_with_cameras(tmp_path):
    # Mock mujoco
    mock_model = MagicMock()
    mock_model.ncam = 2
    mock_data = MagicMock()

    with (
        patch("mujoco.MjModel.from_xml_string", return_value=mock_model),
        patch("mujoco.MjData", return_value=mock_data),
        patch("mujoco.mj_forward"),
        patch("mujoco.Renderer") as mock_renderer_cls,
    ):
        mock_renderer = mock_renderer_cls.return_value
        mock_renderer.render.return_value = MagicMock()

        output_prefix = str(tmp_path / "test")
        xml = "<mujoco/>"

        with patch(
            "src.generators.benchmark.renderer.Image.fromarray"
        ) as mock_fromarray:
            mock_img = mock_fromarray.return_value

            paths = render_scenario(xml, output_prefix)

            assert len(paths) == 2
            assert f"{output_prefix}_cam0.png" in paths
            assert f"{output_prefix}_cam1.png" in paths
            assert mock_renderer.update_scene.call_count == 2
            assert mock_img.save.call_count == 2


def test_render_scenario_failure(tmp_path):
    with patch("mujoco.MjModel.from_xml_string", side_effect=Exception("Invalid XML")):
        output_prefix = str(tmp_path / "test")
        paths = render_scenario("invalid", output_prefix)
        assert paths == []
