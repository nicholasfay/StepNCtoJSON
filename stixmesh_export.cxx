/* $RCSfile: stixmesh_export.cxx,v $
 * $Revision: 1.18 $ $Date: 2015/05/06 15:20:51 $
 * Auth: Jochen Fritz (jfritz@steptools.com)
 * 
 * 	Copyright (c) 1991-2015 by STEP Tools Inc.
 * 	All Rights Reserved
 * 
 * 	This software is furnished under a license and may be used and
 * 	copied only in accordance with the terms of such license and with
 * 	the inclusion of the above copyright notice.  This software and
 * 	accompanying written materials or any other copies thereof may
 * 	not be provided or otherwise made available to any other person.
 * 	No title to or ownership of the software is hereby transferred.
 * 
 * 		----------------------------------------
 */

#include <stp_schema.h>
#include <RoseXMLWriter.h>

#include <stix_asm.h>
#include <stix_unit.h>
#include <stix_context.h>
#include <stix_measure.h>
#include <cstdlib>
#include <iostream>

#include "stixmesh_export.h"

#include <math.h>

#include "stixmesh_error.h"
#include "stixmesh_nurbs.h"
#include "stixmesh_writer.h"
#include "stixmesh_present.h"
#include "stixmesh_simple.h"

/*****************************************************/

static FILE * open_dir_file(const char * dir, const char * fname)
{
    RoseStringObject path = dir;
    path.cat("/");
    path.cat(fname);

    return fopen(path, "w");
}

static void append_double(RoseXMLWriter * xml, double val)
{
    char buff[20];
    sprintf(buff, "%g", val);
    xml->text(buff);
}


static void write_shell(
    StixMeshXMLExport * st,
    stp_representation_item * it,
    const StixMeshStp * shell
    );

void StixMeshXMLExport::appendShell(
    stp_representation * rep,
    stp_representation_item * ri)
{
    if (!StixMeshStpBuilder::canMake(rep, ri))
	return;

    getAsync()->startMesh(rep, ri, opts);
}

int StixMeshXMLExport::flushCompletedShells(int wait)
{
    StixMeshStp * mesh;
    int count = 0;
    while ((mesh = getAsync()->getResult(wait)) != 0) {
	count++;
	reportProgress(mesh->getStepSolid());

	write_shell(this, mesh->getStepSolid(), mesh);

	delete mesh;
    }
    
    return getAsync()->hasRunningJobs();
}


void StixMeshXMLExport::reportProgress(stp_representation_item * it)
{
    count++;    
    stixmesh_ec()->message(
	"Finished facet: shell #%lu (%u/%u)", it->entity_id(),
	count, size);
    fflush(stdout);
}


/*****************************************************/

static void append_bbox(
    RoseXMLWriter * xml,
    const char * name,
    StixMeshBoundingBox * bbox)
{
    xml->beginAttribute(name);

    append_double(xml, bbox->minx);
    xml->text(" ");
    append_double(xml, bbox->miny);
    xml->text(" ");
    append_double(xml, bbox->minz);
    xml->text(" ");
    append_double(xml, bbox->maxx);
    xml->text(" ");
    append_double(xml, bbox->maxy);
    xml->text(" ");
    append_double(xml, bbox->maxz);
    
    xml->endAttribute();
}



void StixMeshXMLExport::appendRef (
    RoseXMLWriter * xml,
    const char * name,
    RoseObject * val)
{
    if (!val)
	return;

    char buff[20];
    sprintf (buff, "id%lu", val->entity_id());

    xml->addAttribute (name, buff);
}

void StixMeshXMLExport::appendVal (
    RoseXMLWriter * xml,
    const char * name,
    double val)
{
    char buff[20];
    sprintf (buff, "%g", val);

    xml->addAttribute (name, buff);
}




/************************************************************/

static void append_shell_rep(
    RoseXMLWriter * xml,
    unsigned * count,
    RoseObject * obj)
{
    if (*count == 0) 
	xml->beginAttribute ("shell");
	
    (*count)++;

    if (*count > 1)
	xml->text (" ");
	
    char buff[20];
    sprintf (buff, "id%lu", obj->entity_id());
    xml->text(buff);
}


void StixMeshXMLExport::appendShellRefs(
    RoseXMLWriter * xml,
    stp_representation * sr)
{
    SetOfstp_representation_item * items = sr->items();
    unsigned sz = items->size();

    unsigned count=0;
    
    for (unsigned i=0; i<sz; i++) {
	stp_representation_item * it = items->get(i);

	if (!StixMeshStpBuilder::isShell(sr, it))
	    continue;

	append_shell_rep(xml, &count, it);	
    }

    if (count > 0) {
	xml->endAttribute();
    }
}

int stixmesh_has_shell(stp_shape_representation * sr)
{
    SetOfstp_representation_item * items = sr->items();
    unsigned sz = items->size();

    for (unsigned i=0; i<sz; i++) {
	stp_representation_item * it = items->get(i);
	if (StixMeshStpBuilder::isShell(sr, it))
	    return ROSE_TRUE;
    }

    return ROSE_FALSE;    
}



static void append_asm_child(
    StixMeshXMLExport * exp,  
    RoseXMLWriter * xml,
    RoseObject * rel
    )
{
    StixMgrAsmRelation * rm = StixMgrAsmRelation::find(rel);
    if (!rm) {
	stixmesh_ec()->error ("Could not find RM\n");
	return;
    }

    stp_representation * child = rm->child;

    xml->beginElement("child");
    exp->appendRef (xml, "ref", child);
    
    
    StixMtrx xform = stix_get_transform(rm);

    xml->beginAttribute ("xform");
    
    unsigned i,j;
    for (i=0; i<4; i++)
	for (j=0; j<4; j++) {
	    char buff[20];
	    sprintf (buff, "%s%g", (i==0&&j==0)?"":" ", xform.get(j,i));
	    xml->text (buff);
	}
    xml->endAttribute();
    
    xml->endElement("child");
}


static void dump_shape_child(
    StixMeshXMLExport * st,
    RoseXMLWriter * xml,
    RoseObject * rel,
    rose_uint_vector * roots)
{
    StixMgrAsmRelation * rm = StixMgrAsmRelation::find(rel);
    if (!rm) {
	stixmesh_ec()->error ("Could not find RM\n");
	return;
    }

    stp_representation * child = rm->child;

    st->dumpShapeAsm(xml, child, roots);
}

static unsigned count_shape_child(
//    StixMeshXMLExport * exp,
    RoseObject * rel,
    rose_uint_vector * roots)
{
    StixMgrAsmRelation * rm = StixMgrAsmRelation::find(rel);
    if (!rm) {
	stixmesh_ec()->error ("Could not find RM\n");
	return 0;
    }

    stp_representation * child = rm->child;

    return stixmesh_count_shapes_asm(child, roots);
}



static void get_shell_bbox(
    StixMeshBoundingBox * bbox,
    const StixMeshFacetSet * shell)
{
    unsigned sz = shell->getVertexCount();
    for (unsigned i=0; i<sz; i++) {
	const double * pt = shell->getVertex(i);
	bbox->update(pt);
    }
}


static void append_area(RoseXMLWriter * xml, const StixMeshStp * shell) {

    double area = 0.;
    
    for (unsigned i=0, sz=shell->getFaceCount(); i<sz; i++) {
	const StixMeshStpFace * face = shell->getFaceInfo(i);
	area += face->getArea();
    }

    xml->beginAttribute("a");
    append_double(xml, area);
    xml->endAttribute();    
}


static void write_shell(
    StixMeshXMLExport * st,
    stp_representation_item * it,
    const StixMeshStp * shell
    )
{
    if (!shell) return;
    
    unsigned color = stixmesh_get_color(it);

    char id[100];
    sprintf (id, "id%lu", shell->getStepSolid()->entity_id());

    RoseXMLWriter * xml = st->getMainXML();

    const char * out_dir = st->getOutputDir();

    if (!out_dir) {
	stixmesh_save_stp_shell(xml, shell, id, color);
    }
    else {
	
	StixMeshBoundingBox bbox;
	get_shell_bbox(&bbox, shell->getFacetSet());

	RoseStringObject fname = "shell_";
	fname.cat(id);
	fname.cat (".xml");

	char sz_buff[10];
	sprintf (sz_buff, "%d", shell->getFacetSet()->getFacetCount());
	
	xml->beginElement("shell");
	xml->addAttribute("id", id);
	xml->addAttribute("size", sz_buff);
	append_bbox(xml, "bbox", &bbox);
	append_area(xml, shell);
	xml->addAttribute("href", fname);
	
	xml->endElement("shell");

	
	FILE * fd = open_dir_file(out_dir, fname);

	RoseOutputFile xmlfile (fd, fname);
	RoseXMLWriter shell_xml(&xmlfile);
	shell_xml.escape_dots = ROSE_FALSE;
	shell_xml.writeHeader();

	unsigned sz = shell->getFaceCount();
	std::cout << "THIS IS THE FACE COUNT" << sz << std::endl;
	system("pause");
	//stixmesh_save_stp_shell(&shell_xml, shell, id, color);

	shell_xml.close();
	xmlfile.flush();
	fclose(fd);
    }
}




void StixMeshXMLExport::appendShells(
    stp_representation * sr)
{
    SetOfstp_representation_item * items = sr->items();
    unsigned sz = items->size();

    for (unsigned i=0; i<sz; i++) {
	appendShell(sr, items->get(i));
    }    
    
}

static unsigned count_shells(
    stp_representation * sr)
{
    unsigned ret = 0;
    
    SetOfstp_representation_item * items = sr->items();
    unsigned sz = items->size();

    for (unsigned i=0; i<sz; i++) {
	if (StixMeshStpBuilder::canMake(sr, items->get(i)))
	    ret++;
    }
    return ret;
}


static void append_annotation_refs(
    RoseXMLWriter * xml,
    stp_representation * sr)
{
    /* FIXME - this generates both constructive geometry and annotations.
     * theses should be split out, but are are not (currently) doing so since
     * that would require updating the webgl javascript.
     */
    
    if (!sr->isa(ROSE_DOMAIN(stp_shape_representation)))
	return;

    StixMeshRepresentationVec * models = stixmesh_get_draughting_models(
	ROSE_CAST(stp_shape_representation, sr));

    StixMeshConstructiveGeomVec * cgeom = stixmesh_get_constructive_geometry(sr);
        
    if (!models && !cgeom)
	return;

    xml->beginAttribute("annotation");
    
    if (models) {
	unsigned sz = models->size();
	if (sz > 0) {
    
	    for (unsigned i=0; i<sz; i++) {
		stp_representation * model = models->get(i);

		if (i > 0)
		    xml->text (" ");

		char buff[20];
		sprintf (buff, "id%lu", model->entity_id());
		xml->text(buff);
	    }
	}
    }

    if (cgeom) {
	unsigned sz = cgeom->size();
	if (sz > 0) {
    
	    for (unsigned i=0; i<sz; i++) {
		stp_constructive_geometry_representation  * rep = cgeom->get(i);

		if (i > 0)
		    xml->text (" ");

		char buff[20];
		sprintf (buff, "id%lu", rep->entity_id());
		xml->text(buff);
	    }
	}
    }
    
    xml->endAttribute();
}



static void write_poly_point(RoseXMLWriter * xml, ListOfDouble * vals)
{
    xml->beginElement("p");

    xml->beginAttribute ("l");
    char buff[20];
    for (int i=0; i<3; i++) {
	if (i > 0)
	    xml->text(" ");
	sprintf (buff, "%g", vals->get(i));
	xml->text(buff);
    }
    xml->endAttribute();
    
    xml->endElement("p");
}

static void write_poly_point(RoseXMLWriter * xml, double vals[3])
{
    xml->beginElement("p");

    xml->beginAttribute ("l");
    char buff[20];
    for (int i=0; i<3; i++) {
	if (i > 0)
	    xml->text(" ");
	sprintf (buff, "%g", vals[i]);
	xml->text(buff);
    }
    xml->endAttribute();
    
    xml->endElement("p");
}


static void append_step_curve(RoseXMLWriter * xml, stp_curve * c)
{
    /* May also want to handle composite curves here in this routine, since
     * a composite curve may contain via points, and thus run into trouble.*/
    unsigned i,sz;

    if (c->isa(ROSE_DOMAIN(stp_polyline))) {
	
	stp_polyline * poly = ROSE_CAST(stp_polyline, c);
	ListOfstp_cartesian_point * pts = poly->points();
	
	if (!pts || pts->size() < 2) 
	    return;

	xml->beginElement("polyline");
	
	for (i=0, sz=pts->size(); i<sz; i++) {
	    stp_cartesian_point * pt = pts->get(i);
	    ListOfDouble * vals = pt->coordinates();
	    write_poly_point(xml, vals);
	}

	xml->endElement("polyline");
    }

}

static void export_annotation(RoseXMLWriter * xml,
			      stp_geometric_set * gset)
{
    SetOfstp_geometric_set_select * elems = gset->elements();
    if (!elems) return;

    unsigned i,sz;
    for (i=0, sz=elems->size(); i<sz; i++) {
	stp_geometric_set_select * sel = elems->get(i);
	if (sel-> is_curve()) {
	    append_step_curve(xml, sel->_curve());
	}
	else if (sel-> is_point()) {
	}
	else if (sel-> is_surface()) {
	}
	
    }
}

static void export_annotation(RoseXMLWriter * xml,
			      stp_representation_item * it)
{
    if (!it)
	return;

    if (it-> isa(ROSE_DOMAIN(stp_annotation_plane)))
    {
	// do we do anything special with the plane?
	stp_annotation_plane * ap = ROSE_CAST(stp_annotation_plane,it);
	unsigned i, sz;

	for (i=0, sz=ap->elements()->size(); i<sz; i++) {
	    // either draughting_callout or styled_item, both of which
	    // are rep items.
	    stp_representation_item * elem = 
		ROSE_CAST(stp_representation_item,
			  rose_get_nested_object(ap-> elements()-> get(i)));

	    export_annotation (xml, elem);
	}
    }
    
    else if (it-> isa(ROSE_DOMAIN(stp_annotation_occurrence)))
    {
	
	stp_annotation_occurrence * ao
	    = ROSE_CAST(stp_annotation_occurrence,it);
	
	if (!ao-> item()) return;

	if (ao-> item()-> isa(ROSE_DOMAIN(stp_geometric_set))) {
	    export_annotation(xml, ROSE_CAST(stp_geometric_set,ao->item()));
	}
    }

    else {
	stixmesh_ec()->warning ("export_annotation unimplemented case: %s\n",
				it->domain()->name());
    }
    
}


static void export_model_body(
    StixMeshXMLExport * exp,
    RoseXMLWriter * xml,
    stp_representation * model)
{
    xml->beginElement("annotation");
    exp->appendRef (xml, "id", model);
    
    SetOfstp_representation_item * items = model->items();
    unsigned sz = items->size();
    for (unsigned i=0; i<sz; i++) {
	stp_representation_item * it = items->get(i);
	export_annotation(xml, it);
    }

    xml->endElement("annotation");
}


static void export_model(
    StixMeshXMLExport * st,
    RoseXMLWriter * xml,
    stp_representation * model)
{
    if (model->wasVisited())
	return;
    model->visit();

    const char * out_dir = st->getOutputDir();
    
    if (!out_dir)
	export_model_body(st, xml, model);
    else {

	char id[100];
	sprintf (id, "id%lu", model->entity_id());
	
	RoseStringObject fname = "annotation_";
	fname.cat(id);
	fname.cat (".xml");

	xml->beginElement("annotation");
	st->appendRef (xml, "id", model);
	xml->addAttribute("href", fname);
	xml->endElement("annotation");

	FILE * fd = open_dir_file(out_dir, fname);

	RoseOutputFile xmlfile (fd, fname);
	RoseXMLWriter part_xml(&xmlfile);
	part_xml.escape_dots = ROSE_FALSE;
	part_xml.writeHeader();

	export_model_body(st, &part_xml, model);

	part_xml.close();
	xmlfile.flush();
	fclose(fd);
    }
}


static void append_step_curve(
    RoseXMLWriter * xml,
    stp_representation * rep,
    stp_bounded_curve * curve)
{
    /* FIXME: merge this code with SGBoundedCurve ctor. */

    StixMeshNurbs nurbs;
    stixmesh_create_bounded_curve(&nurbs, curve, rep);

    StixMeshBoundingBox bbox;
    
    if (!nurbs.getConvexHull(&bbox)) {
	//DPRINTF (("Could not get convec hull of curve, skipping"));
	return;
    }
    double tol = bbox.diagonal() / 100.;

    rose_real_vector u_vals;
    nurbs.extractTolerancedPoints(&u_vals, tol, 1);

    xml->beginElement("polyline");
    
    unsigned i,sz;
    for (i=0, sz=u_vals.size(); i<sz; i++) {
	double xyz[3];
	nurbs.eval(xyz, u_vals[i]);
	write_poly_point(xml, xyz);
    }

    xml->endElement("polyline");    
}

static void export_rep_item(
    RoseXMLWriter * xml,
    stp_representation * rep,
    stp_representation_item * it)
{
    if (!it)
	return;

    if (it->isa(ROSE_DOMAIN(stp_bounded_curve))) {
	append_step_curve(xml, rep, ROSE_CAST(stp_bounded_curve, it));
    }

}

static void export_constructive_geom_body(
    StixMeshXMLExport * exp,
    RoseXMLWriter * xml,
    stp_constructive_geometry_representation * cgr)
{
    xml->beginElement("annotation");
    exp->appendRef (xml, "id", cgr);
    
    SetOfstp_representation_item * items = cgr->items();
    unsigned sz = items->size();
    for (unsigned i=0; i<sz; i++) {
	stp_representation_item * it = items->get(i);
	export_rep_item(xml, cgr, it);
    }

    xml->endElement("annotation");
}


static void export_constructive_geom(
    StixMeshXMLExport * exp,
    RoseXMLWriter * xml,
    stp_constructive_geometry_representation * cg)
{
    if (cg->wasVisited())
	return;
    cg->visit();

    const char * out_dir = exp->getOutputDir();
    
    if (!out_dir)
	export_constructive_geom_body(exp, xml, cg);
    else {

	char id[100];
	sprintf (id, "id%lu", cg->entity_id());
	
	RoseStringObject fname = "constructive_";
	fname.cat(id);
	fname.cat (".xml");

	xml->beginElement("annotation");
	exp->appendRef (xml, "id", cg);
	xml->addAttribute("href", fname);
	xml->endElement("annotation");


	FILE * fd = open_dir_file(out_dir, fname);

	RoseOutputFile xmlfile (fd, fname);
	RoseXMLWriter part_xml(&xmlfile);
	part_xml.escape_dots = ROSE_FALSE;
	part_xml.writeHeader();

	export_constructive_geom_body(exp, &part_xml, cg);

	part_xml.close();
	xmlfile.flush();
	fclose(fd);
    }
    
}



static void append_annotations(
    StixMeshXMLExport * st,
    RoseXMLWriter * xml,
    stp_representation * sr)
{
    if (!sr->isa(ROSE_DOMAIN(stp_shape_representation)))
	return;

    StixMeshRepresentationVec * models = stixmesh_get_draughting_models(
	ROSE_CAST(stp_shape_representation, sr));

    StixMeshConstructiveGeomVec * cgeom = stixmesh_get_constructive_geometry(sr);
        
    if (!models && !cgeom)
	return;

    if (models) {
	unsigned sz = models->size();

	for (unsigned i=0; i<sz; i++) {
	    stp_representation * model = models->get(i);
	    
	    export_model(st, xml, model);
	}
    }

    if (cgeom) {
	unsigned sz = cgeom->size();

	for (unsigned i=0; i<sz; i++) {
	    stp_constructive_geometry_representation * cg = cgeom->get(i);
	    
	    export_constructive_geom(st, xml, cg);
	}
    }
    
}


void StixMeshXMLExport::dumpShapeAsm(
    RoseXMLWriter * xml,
    stp_representation * sr,
    rose_uint_vector * roots)
{
    unsigned i, j, sz;

    if (sr->wasVisited()) return;
    sr->visit();

    StixMgrAsmShapeRep * srm = StixMgrAsmShapeRep::find(sr);
    if (!srm) {
	stixmesh_ec()->error ("Could not find RM\n");
	return;
    }

    int found;
        
    if (roots == 0)
	found = ROSE_TRUE;
    else {
	found = ROSE_FALSE;
	unsigned sz =roots->size();
	
	unsigned eid = sr->entity_id();
	
	for (i=0; i<sz; i++) {
	    unsigned e = roots->get(i);
	    if (e == eid) {
		found = ROSE_TRUE;
		break;
	    }
	}	
    }    

    xml->beginElement("shape");
    appendRef (xml, "id", sr);

    StixUnit unit = stix_get_context_length_unit(sr);
    if (unit != stixunit_unknown) {
	xml->beginAttribute("unit");
	xml->text (stix_get_unit_name(unit));
	char buff[20];
	sprintf (buff, " %f", stix_get_converted_measure(1., unit, stixunit_m));
	xml->text(buff);    
	xml->endAttribute();
    }
    
    if (found)
	appendShellRefs(xml, sr);

    append_annotation_refs(xml, sr);
    
    for (j=0, sz=srm->child_rels.size(); j<sz; j++) 
	append_asm_child(this, xml, srm->child_rels[j]);
    for (j=0, sz=srm->child_mapped_items.size(); j<sz; j++) 
	append_asm_child(this, xml, srm->child_mapped_items[j]);

    xml->endElement("shape");

    for (j=0, sz=srm->child_rels.size(); j<sz; j++) 
	dump_shape_child(this, xml, srm->child_rels[j], roots);

    for (j=0, sz=srm->child_mapped_items.size(); j<sz; j++) 
	dump_shape_child(this, xml, srm->child_mapped_items[j], roots);

    if (found) {
	appendShells(sr);
	flushCompletedShells(1);
	append_annotations(this, xml, sr);
    }
}

unsigned stixmesh_count_shapes_asm(
    stp_representation * sr, rose_uint_vector * roots)
{
    unsigned ret=0;
    unsigned i, j, sz;

    if (sr->wasVisited()) return 0;
    sr->visit();

    StixMgrAsmShapeRep * srm = StixMgrAsmShapeRep::find(sr);
    if (!srm) {
	stixmesh_ec()->error ("Could not find RM\n");
	return 0;
    }

    int found;
        
    if (roots == 0)
	found = ROSE_TRUE;
    else {
	found = ROSE_FALSE;
	unsigned sz =roots->size();
	
	unsigned eid = sr->entity_id();
	
	for (i=0; i<sz; i++) {
	    unsigned e = roots->get(i);
	    if (e == eid) {
		found = ROSE_TRUE;
		break;
	    }
	}	
    }    
    
    for (j=0, sz=srm->child_rels.size(); j<sz; j++) {
//	dump_shape_child(xml, out_dir, srm->child_rels[j], roots);
	ret += count_shape_child(srm->child_rels[j], roots);
    }

    for (j=0, sz=srm->child_mapped_items.size(); j<sz; j++) {
//	dump_shape_child(xml,out_dir,  srm->child_mapped_items[j], roots);
	ret += count_shape_child(srm->child_mapped_items[j], roots);
    }

    if (found) {
//	stixmesh_append_shells (xml, out_dir, sr);
	ret += count_shells (sr);
    }

    return ret;
}




#if 0
void StixMeshXMLExport::stixmesh_xml_dump_shape_asm(
//    RoseXMLWriter * xml,
    stp_representation * sr,
    rose_uint_vector * roots)
{
    StixMeshXMLExport st(xml, 0);
    
    stixmesh_xml_dump_shape_asm(&st, sr, roots);
}
#endif
