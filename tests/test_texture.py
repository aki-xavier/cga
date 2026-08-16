from pathlib import Path

import mlx.core as mx
from PIL import Image

from cga.engine import MeshBasicMaterial, Texture
from cga.scene_lang import SceneLoader


def test_texture_decodes_srgb_and_bilinearly_samples():
    texture = Texture.from_rgba(
        [
            [[1.0, 0.0, 0.0, 1.0], [0.0, 1.0, 0.0, 1.0]],
            [[0.0, 0.0, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0]],
        ]
    )
    sampled = texture.sample(mx.array([[0.5, 0.5]], dtype=mx.float32), "clamp", "clamp")
    # Bilinear filtering averages the four corners in linear color space.
    assert mx.allclose(sampled[0, :3], mx.array([0.5, 0.5, 0.5]), atol=1e-5).item()


def test_texture_repeat_wraps_integer_uvs():
    texture = Texture.from_rgba([[[1.0, 0.0, 0.0, 1.0]]])
    a = texture.sample(mx.array([[0.25, 0.25]], dtype=mx.float32))
    b = texture.sample(mx.array([[1.25, -0.75]], dtype=mx.float32))
    assert mx.array_equal(a, b).item()


def test_cgs_material_map_resolves_against_asset_root(tmp_path: Path):
    path = tmp_path / "albedo.png"
    Image.new("RGBA", (1, 1), (255, 0, 0, 255)).save(path)
    scene, _camera = SceneLoader.load(
        'material(map="albedo.png", unlit=true) sphere(r=1);', asset_root=tmp_path
    )
    material = scene.objects[0].material
    assert isinstance(material, MeshBasicMaterial)
    assert material.map is not None
    assert material.map.width == 1 and material.map.height == 1
