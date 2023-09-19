#pragma once

#include <deque>
#include <cgv/base/node.h>
#include <cgv/data/data_format.h>
#include <cgv/math/fvec.h>
#include <cgv/math/fmat.h>
#include <cgv/media/color.h>
#include <cgv/render/texture.h>
#include <cgv/render/drawable.h>
#include <cgv/render/view.h>
#include <cgv/render/shader_program.h>
#include <cgv/render/attribute_array_binding.h>
#include <cgv/gui/provider.h>
#include <cgv/gui/event_handler.h>
#include <cgv_gl/point_renderer.h>
#include <plot/plot2d.h>

#include "lib_begin.h"

enum GroundTruthMode
{
	GTM_NORMAL,
	GTM_T
};

enum TubeGenerationMode
{
	TGM_FIXED,
	TGM_RANDOM,
	TGM_CURVE_FAMILY,
	TGM_STREAM_TUBES
};

enum TubeColoringMode
{
	TCM_TUBE,
	TCM_MAP_T,
	TCM_MAP_ARC_LENGTH,
	TCM_END,
	TCM_BEGIN = TCM_TUBE
};

enum TubeRenderMode
{
	TRM_HIDE = -1,
	TRM_SPHERE,
	TRM_ELLIPSOID,
	TRM_CYLINDER,
	TRM_CONE,
	TRM_TESSELATION,
	TRM_RAYCASTING,
	TRM_END,
	TRM_BEGIN = TRM_HIDE
};

class CGV_API tubes_view : 
	public cgv::base::node, 
	public cgv::render::drawable, 
	public cgv::gui::event_handler, 
	public cgv::gui::provider
{
public:
	// types
	typedef float crd_type;
	typedef cgv::math::fvec<crd_type, 2> vec2;
	typedef cgv::math::fvec<crd_type, 3> vec3;
	typedef cgv::math::fvec<crd_type, 4> vec4;
	typedef cgv::math::fmat<crd_type, 2, 2> mat2;
	typedef cgv::math::fmat<crd_type, 3, 3> mat3;
	typedef cgv::math::fmat<crd_type, 3, 2> mat3x2;
	typedef cgv::math::fmat<crd_type, 4, 4> mat4;
	typedef cgv::media::color<crd_type,cgv::media::RGB,cgv::media::OPACITY> clr_type;
protected:
	std::vector<vec3> samples;
	std::vector<clr_type> sample_colors;
	std::vector<float> sample_times;
	cgv::media::axis_aligned_box<float, 3> sample_box;
	unsigned subsampling_factor;
	void extract_spline_from_samples();

	cgv::render::point_render_style prs;
	cgv::render::point_renderer pr;
	bool show_samples;
	void draw_samples(cgv::render::context& ctx);

	std::vector<vec4> positions;
	std::vector<vec3> parameters; // size, arc_length, inverse arc length
	std::vector<clr_type> colors;
	std::vector<mat3x2> tensors;
	std::vector<float> spline_times;

	std::vector<vec4> position_derivatives;
	std::vector<vec3> parameters_derivatives;
	std::vector<vec4> color_derivatives;
	std::vector<mat3x2> tensor_derivatives;

	std::vector<unsigned> strips;

	TubeRenderMode render_mode;
	unsigned nr_samples_per_segment;
	bool use_inv_arc_length_sampling;
	unsigned color_map_resolution;
	TubeColoringMode coloring_mode;
	TubeGenerationMode tube_generation_mode;
	void generate_fixed_tubes();
	void clear_tubes();
	void on_new_dataset();
	void generate_random_tubes();
	bool read_kml_tubes(const std::string& file_name);
	bool read_hermite_tubes(const std::string& file_name);
	bool read_bezier_tubes(const std::string& file_name);
	unsigned nr_tube_strips;
	bool illuminate;
	bool show_silhouette;
	bool silhouette_only;
	clr_type silhouette_color;
	bool show_backside;
	float scale;
	GroundTruthMode ground_truth_mode;
	cgv::render::texture color_map;
	bool interpolate_color_map;
	int sign(crd_type v) const;
	crd_type find_zero(crd_type t0, crd_type v0, crd_type t1, crd_type v1, unsigned diff, crd_type ds_coeffs[28], unsigned* iters_ptr = 0) const;
	crd_type find_zero_oracle(crd_type t0, crd_type v0, crd_type t1, crd_type v1, const vec3& czDiff, crd_type ds_coeffs[28], unsigned* iters_ptr) const;
	void propagate_limits(std::vector<std::pair<tubes_view::crd_type, int> >& limits, unsigned diff, crd_type ds_coeffs[28]) const;
	crd_type eval_s(crd_type t, const vec4& cz, crd_type ds_coeffs[28]) const;
	crd_type eval_ds(crd_type t, unsigned diff, crd_type ds_coeffs[28]) const;
	crd_type eval_oracle(crd_type t, const vec3& czDiff, crd_type ds_coeffs[28]) const;
	vec3 compute_bezier_derivative(const vec4& B) const;
	vec4 eval_bezier_diffs(crd_type t, const vec4& B) const;
	crd_type eval_bezier(crd_type t, const vec3& B) const;
	crd_type eval_bezier(crd_type t, const vec4& B) const;
	void compute_ds_coefficients(const vec4& cx, const vec4& cy, const vec4& cz, const vec4& r, crd_type result[28]) const;
	unsigned add_plot(const std::string& name, float r, float g, float b);
	unsigned plot_resolution;
	int plot_width, plot_height, plot_offset_x, plot_offset_y;
	crd_type scale_base;
	bool debug_ray_intersection;
//	void compute_ds_plot(unsigned pi, const vec4& cx, const vec4& cy, const vec4& r);
	void compute_bezier_plot(unsigned pi, const vec4& B);
	void compute_monom_plot(unsigned pi, int diff, crd_type coeffs[28]);
	void compute_view_dependent_plots(const vec3& ray_eye, int current_segment);
	void ensure_plots();
	void compute_view_independent_plots(int current_segment);
	void compute_ray_plots(const vec3& eye, const vec3& v, const vec4& cx, const vec4& cy, const vec4& cz, const vec4& r, crd_type ds_coeffs[28], bool curve = true, bool ds = true, bool s = true);
	//void recompute_plots(const vec4& cx, const vec4& cy, const vec4& cz, const vec4& r, crd_type ds_coeffs[28]);
	crd_type consider_t(crd_type t, const vec4& cz, crd_type coeffs[28], crd_type& s, crd_type& t_s, bool& found_s) const;
	void get_bezier_tube(int current_segment, cgv::math::fmat<crd_type, 4, 4>& C, vec4& r, cgv::math::fmat<crd_type, 4, 4>* col_mat_ptr = 0) const;
	void get_ray_space_bezier_polygons(const cgv::math::fmat<crd_type, 4, 4>& C, const vec3& v, const vec3& x, const vec3& ray_eye, vec4& cx, vec4& cy, vec4& cz) const;
	void compute_intersection_intervals(crd_type ds_coeffs[28], std::vector<std::pair<crd_type, int> >& limits) const;
	bool find_first_intersection(crd_type ds_coeffs[28], std::vector<std::pair<crd_type, int> >& limits, const vec4& cz, crd_type& s, crd_type& t_s) const;

	vec4 last_cx, last_cy, last_cz, last_r;
	crd_type last_ds_coeffs[28];
	void analyze_ray(const vec3& eye, const vec3& v, const vec3& u);

	vec3 last_eye, last_v, last_u, last_rendered_eye;
	void pick_ray(int x, int y);
private:
	bool found_s;
	crd_type s;
	crd_type t_s;
	
	int plot_curve_indices[7];
	int plot_ds_indices[7];
	int plot_s_index, plot_ds_tilde_index, plot_r_tilde_index;

	bool auto_compute_plots;
	bool compute_curve_plots, compute_curve_plot[7];
	bool compute_ds_plots, compute_ds_plot[7];
	bool compute_s_plot, compute_ds_tilde_plot, compute_r_tilde_plot;

	crd_type eps_sign, eps_zero_scale;

	bool show_plot;
	bool color_map_changed;
	bool tubes_changed;
	dmat4 DPV;
	cgv::render::view* view_ptr;
	bool ensure_view_ptr();

	unsigned current_segment;
	cgv::plot::plot2d plot;
	void adjust_plots();

	bool show_tesselation;
	bool tesselate_cylinder;
	bool tesselate_silhouette;
	bool tesselate_only_current;
	crd_type tesselation_safety;
	bool tesselate_correct_normal;
	bool tesselation_wireframe;
	int radial_subdivisions;

	void draw_tesselation(cgv::render::context& ctx, int current_segment, const vec3& eye);

protected:
	/**@name variables used to adjust current tube vertex*/
	//@{
	unsigned current_tube_vertex_index;
	vec4 current_position;
	vec3 current_parameters;
	clr_type current_color;
	mat3x2 current_tensor;
	vec4 current_position_derivative;
	vec3 current_parameters_derivative;
	vec4 current_color_derivative;
	mat3x2 current_tensor_derivative;
	//@}

	cgv::render::shader_program sphere_prog;
	cgv::render::shader_program ellipsoid_prog;
	cgv::render::shader_program cylinder_prog;
	cgv::render::shader_program tesselate_prog;
	cgv::render::shader_program raycast_prog;
	cgv::render::attribute_array_binding aab_sphere_prog;
	typedef cgv::media::color<cgv::type::uint8_type> color3b;
	bool show_ground_truth;
	std::vector<color3b> ground_truth_rendering;
	void compute_ground_truth();
	template <typename T>
	void set_attribute_array(cgv::render::context& ctx, const std::string& name, cgv::render::shader_program& prog, const T* arr, size_t count, size_t stride)
	{
		int al = sphere_prog.get_attribute_location(ctx, name);
		if (al == -1) {
			std::cerr << "could not find attribute location " << al << " in shader program!" << std::endl;
			abort();
		}
		aab_sphere_prog.set_attribute_array(ctx, al, arr, count, stride);
	}
	void generate_tubes();
public:
	/// construct with initialized mesh
	tubes_view();
	/// generate simple tube
	/// update
	void on_set(void* member_ptr);
	/// overload methods of drawable
	std::string get_type_name() const { return "tubes_view"; }
	///
	void resize(unsigned int w, unsigned int h);
	///
	bool init(cgv::render::context& ctx);
	///
	void init_frame(cgv::render::context& ctx);
	///
	void draw(cgv::render::context& ctx);
	///
	void clear(cgv::render::context& ctx);
	/// overload and implement this method to handle events
	bool handle(cgv::gui::event& e);
	/// overload to stream help information to the given output stream
	void stream_help(std::ostream& os);
	void stream_stats(std::ostream& os);
	///
	void create_gui();
};

#include <cgv/config/lib_end.h>